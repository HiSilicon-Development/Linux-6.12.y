// SPDX-License-Identifier: GPL-2.0
/* Copyright 2019 Linaro, Ltd, Rob Herring <robh@kernel.org> */
/* Copyright 2019 Collabora ltd. */
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/dma-resv.h>
#include <drm/gpu_scheduler.h>
#include <drm/panfrost_drm.h>

#include "panfrost_device.h"
#include "panfrost_devfreq.h"
#include "panfrost_job.h"
#include "panfrost_features.h"
#include "panfrost_issues.h"
#include "panfrost_gem.h"
#include "panfrost_regs.h"
#include "panfrost_gpu.h"
#include "panfrost_mmu.h"
#include "panfrost_dump.h"

#define JOB_TIMEOUT_MS 500

#define job_write(dev, reg, data) writel(data, dev->iomem + (reg))
#define job_read(dev, reg) readl(dev->iomem + (reg))

struct panfrost_queue_state {
	struct drm_gpu_scheduler sched;
	u64 fence_context;
	u64 emit_seqno;
};

struct panfrost_job_slot {
	struct panfrost_queue_state queue[NUM_JOB_SLOTS];
	spinlock_t job_lock;
	int irq;
};

static struct panfrost_job *
to_panfrost_job(struct drm_sched_job *sched_job)
{
	return container_of(sched_job, struct panfrost_job, base);
}

struct panfrost_fence {
	struct dma_fence base;
	struct drm_device *dev;
	/* panfrost seqno for signaled() test */
	u64 seqno;
	int queue;
};

static inline struct panfrost_fence *
to_panfrost_fence(struct dma_fence *fence)
{
	return (struct panfrost_fence *)fence;
}

static const char *panfrost_fence_get_driver_name(struct dma_fence *fence)
{
	return "panfrost";
}

static const char *panfrost_fence_get_timeline_name(struct dma_fence *fence)
{
	struct panfrost_fence *f = to_panfrost_fence(fence);

	switch (f->queue) {
	case 0:
		return "panfrost-js-0";
	case 1:
		return "panfrost-js-1";
	case 2:
		return "panfrost-js-2";
	default:
		return NULL;
	}
}

static const struct dma_fence_ops panfrost_fence_ops = {
	.get_driver_name = panfrost_fence_get_driver_name,
	.get_timeline_name = panfrost_fence_get_timeline_name,
};

static struct dma_fence *panfrost_fence_create(struct panfrost_device *pfdev, int js_num)
{
	struct panfrost_fence *fence;
	struct panfrost_job_slot *js = pfdev->js;

	fence = kzalloc(sizeof(*fence), GFP_KERNEL);
	if (!fence)
		return ERR_PTR(-ENOMEM);

	fence->dev = pfdev->ddev;
	fence->queue = js_num;
	fence->seqno = ++js->queue[js_num].emit_seqno;
	dma_fence_init(&fence->base, &panfrost_fence_ops, &js->job_lock,
		       js->queue[js_num].fence_context, fence->seqno);

	return &fence->base;
}

int panfrost_job_get_slot(struct panfrost_job *job)
{
	/* JS0: fragment jobs.
	 * JS1: vertex/tiler jobs
	 * JS2: compute jobs
	 */
	if (job->requirements & PANFROST_JD_REQ_FS)
		return 0;

/* Not exposed to userspace yet */
#if 0
	if (job->requirements & PANFROST_JD_REQ_ONLY_COMPUTE) {
		if ((job->requirements & PANFROST_JD_REQ_CORE_GRP_MASK) &&
		    (job->pfdev->features.nr_core_groups == 2))
			return 2;
		if (panfrost_has_hw_issue(job->pfdev, HW_ISSUE_8987))
			return 2;
	}
#endif
	return 1;
}

static void panfrost_job_write_affinity(struct panfrost_device *pfdev,
					u32 requirements,
					int js)
{
	u64 affinity;

	/*
	 * Use all cores for now.
	 * Eventually we may need to support tiler only jobs and h/w with
	 * multiple (2) coherent core groups
	 */
	affinity = pfdev->features.shader_present;

	job_write(pfdev, JS_AFFINITY_NEXT_LO(js), lower_32_bits(affinity));
	job_write(pfdev, JS_AFFINITY_NEXT_HI(js), upper_32_bits(affinity));
}

static u32
panfrost_get_job_chain_flag(const struct panfrost_job *job)
{
	struct panfrost_fence *f = to_panfrost_fence(job->done_fence);

	if (!panfrost_has_hw_feature(job->pfdev, HW_FEATURE_JOBCHAIN_DISAMBIGUATION))
		return 0;

	return (f->seqno & 1) ? JS_CONFIG_JOB_CHAIN_FLAG : 0;
}

static struct panfrost_job *
panfrost_dequeue_job(struct panfrost_device *pfdev, int slot)
{
	struct panfrost_job *job = pfdev->jobs[slot][0];

	WARN_ON(!job);
	if (job->is_profiled) {
		if (job->engine_usage) {
			job->engine_usage->elapsed_ns[slot] +=
				ktime_to_ns(ktime_sub(ktime_get(), job->start_time));
			job->engine_usage->cycles[slot] +=
				panfrost_cycle_counter_read(pfdev) - job->start_cycles;
		}
		panfrost_cycle_counter_put(job->pfdev);
	}

	pfdev->jobs[slot][0] = pfdev->jobs[slot][1];
	pfdev->jobs[slot][1] = NULL;

	return job;
}

static unsigned int
panfrost_enqueue_job(struct panfrost_device *pfdev, int slot,
		     struct panfrost_job *job)
{
	if (WARN_ON(!job))
		return 0;

	if (!pfdev->jobs[slot][0]) {
		pfdev->jobs[slot][0] = job;
		return 0;
	}

	WARN_ON(pfdev->jobs[slot][1]);
	pfdev->jobs[slot][1] = job;
	WARN_ON(panfrost_get_job_chain_flag(job) ==
		panfrost_get_job_chain_flag(pfdev->jobs[slot][0]));
	return 1;
}

/*
 * Diagnostic: print the words at a GPU virtual address as the job's own
 * buffers see them.  Called from the submit path and from the job error
 * interrupt (which runs in the threaded handler, before the reset work tears
 * the mappings down), so the two prints can be compared to find out whether
 * the chain changes between submission and execution.
 */
static void panfrost_diag_dump_va(struct panfrost_device *pfdev,
				  struct panfrost_job *job, u64 va,
				  const char *tag)
{
	unsigned int i;

	for (i = 0; i < job->bo_count; i++) {
		struct panfrost_gem_mapping *m = job->mappings[i];
		struct panfrost_gem_object *bo = to_panfrost_bo(job->bos[i]);
		struct iosys_map map;
		u64 start, end, off;
		u32 *w;

		if (!m || bo->is_heap)
			continue;

		start = (u64)m->mmnode.start << PAGE_SHIFT;
		end = start + ((u64)m->mmnode.size << PAGE_SHIFT);
		if (va < start || va >= end)
			continue;

		off = va - start;
		if (drm_gem_vmap_unlocked(&bo->base.base, &map))
			return;
		if (off + 32 <= bo->base.base.size) {
			struct scatterlist *sgl;
			unsigned int pg = off >> PAGE_SHIFT, k = 0;
			u64 bo_phys = 0;
			phys_addr_t as_phys;

			w = (u32 *)((u8 *)map.vaddr + off);
			for (sgl = bo->base.sgt ? bo->base.sgt->sgl : NULL; sgl;
			     sgl = sg_next(sgl)) {
				unsigned int n = sgl->length >> PAGE_SHIFT;

				if (pg < k + n) {
					bo_phys = sg_phys(sgl) +
						  ((u64)(pg - k) << PAGE_SHIFT);
					break;
				}
				k += n;
			}
			as_phys = job->mmu ?
				  job->mmu->pgtbl_ops->iova_to_phys(job->mmu->pgtbl_ops, va) : 0;
			dev_err(pfdev->dev,
				"DIAG %s va=0x%llx BO[%u]+0x%llx active=%d madv=%u pages=%d: %08x %08x %08x %08x %08x %08x %08x %08x\n"
				"DIAG %s pa: as=%pa bo=%pa %s (page granular)\n",
				tag, va, i, off, m->active, bo->base.madv,
				!!bo->base.pages, w[0], w[1], w[2], w[3],
				w[4], w[5], w[6], w[7],
				tag, &as_phys, &bo_phys,
				(((u64)as_phys & PAGE_MASK) == (bo_phys & PAGE_MASK)) ?
					"MATCH" : "MISMATCH");
		}
		drm_gem_vunmap_unlocked(&bo->base.base, &map);
		return;
	}

	dev_err(pfdev->dev, "DIAG %s va=0x%llx is in none of the %u job BOs\n",
		tag, va, job->bo_count);
}

/*
 * Diagnostic: every address a job chain refers to must be mapped by one of the
 * buffers the job carries.  Walk the chain words and report the ones that look
 * like addresses and are covered by nothing - that is what the GPU fails to
 * fetch.  Run in a working and in a failing configuration to compare.
 */
static void panfrost_diag_check_chain(struct panfrost_device *pfdev,
				      struct panfrost_job *job, u64 jc)
{
	struct panfrost_gem_mapping *m;
	struct panfrost_gem_object *bo;
	struct iosys_map map;
	u64 start, off;
	const u32 *w;
	unsigned int i, j, uncovered = 0;

	for (i = 0; i < job->bo_count; i++) {
		m = job->mappings[i];
		bo = to_panfrost_bo(job->bos[i]);
		if (!m || bo->is_heap)
			continue;

		start = (u64)m->mmnode.start << PAGE_SHIFT;
		if (jc < start ||
		    jc >= start + ((u64)m->mmnode.size << PAGE_SHIFT))
			continue;

		off = jc - start;
		if (off + 128 > bo->base.base.size)
			return;
		if (drm_gem_vmap_unlocked(&bo->base.base, &map))
			return;

		w = (const u32 *)((const u8 *)map.vaddr + off);
		for (j = 0; j < 32; j++) {
			u64 va = le32_to_cpu(w[j]);
			unsigned int k;

			if (va < 0x1000 || va > 0xfffff000ULL)
				continue;
			for (k = 0; k < job->bo_count; k++) {
				struct panfrost_gem_mapping *km = job->mappings[k];
				u64 ks, ke;

				if (!km)
					continue;
				ks = (u64)km->mmnode.start << PAGE_SHIFT;
				ke = ks + ((u64)km->mmnode.size << PAGE_SHIFT);
				if (va >= ks && va < ke)
					break;
			}
			if (k == job->bo_count && uncovered++ < 6)
				dev_err(pfdev->dev,
					"DIAG chain UNCOVERED jc=0x%llx word[%u]=0x%08x nbo=%u pid=%d comm=%s\n",
					jc, j, (u32)va, job->bo_count,
					job->diag_pid, job->diag_comm);
		}
		drm_gem_vunmap_unlocked(&bo->base.base, &map);
		if (uncovered)
			dev_err(pfdev->dev,
				"DIAG chain jc=0x%llx uncovered=%u (of 32 words)\n",
				jc, uncovered);
		return;
	}

	dev_err(pfdev->dev,
		"DIAG chain jc=0x%llx not in any of the %u job buffers\n",
		jc, job->bo_count);
}

/*
 * Diagnostic: walk the job chain the way the hardware does - one 64 byte
 * descriptor at a time, following the 64 bit next pointer in words 6:7 - and
 * print every word.  Field layout for v5 (genxml), words are 32 bit:
 *   0 exception status, 1 first incomplete task, 2:3 fault pointer,
 *   4 is64b:1 type:7 barrier inv-cache suppress-prefetch index:16,
 *   5 dependency1:16 dependency2:16, 6:7 next.
 * The fragment job payload starts at word 8: bounds at 8:9, framebuffer
 * pointer at 10:11, tile enable map at 12:13, row stride at 14.  A zero
 * framebuffer pointer sends the GPU to VA 0.
 */
static void panfrost_diag_walk_chain(struct panfrost_device *pfdev,
				     struct panfrost_job *job, u64 jc,
				     const char *tag)
{
	unsigned int hop;

	dev_err(pfdev->dev, "DIAG %s walk enter jc=0x%llx bo_count=%u\n",
		tag, jc, job->bo_count);

	for (hop = 0; hop < 4 && jc; hop++) {
		struct panfrost_gem_mapping *m;
		struct panfrost_gem_object *bo;
		struct iosys_map map;
		u64 start, off, next;
		const u32 *w;
		unsigned int i;

		for (i = 0; i < job->bo_count; i++) {
			m = job->mappings[i];
			bo = to_panfrost_bo(job->bos[i]);
			if (!m || bo->is_heap || !bo->base.base.size)
				continue;
			start = (u64)m->mmnode.start << PAGE_SHIFT;
			if (jc >= start &&
			    jc < start + ((u64)m->mmnode.size << PAGE_SHIFT))
				break;
		}
		if (i == job->bo_count) {
			dev_err(pfdev->dev,
				"DIAG %s hop%u jc=0x%llx not in any job buffer\n",
				tag, hop, jc);
			return;
		}
		off = jc - start;
		if (off + 64 > bo->base.base.size) {
			dev_err(pfdev->dev,
				"DIAG %s hop%u jc=0x%llx BO[%u]+0x%llx past end size=0x%zx\n",
				tag, hop, jc, i, off, bo->base.base.size);
			return;
		}
		if (drm_gem_vmap_unlocked(&bo->base.base, &map)) {
			dev_err(pfdev->dev,
				"DIAG %s hop%u jc=0x%llx BO[%u]+0x%llx vmap failed\n",
				tag, hop, jc, i, off);
			return;
		}

		w = (const u32 *)((const u8 *)map.vaddr + off);
		dev_err(pfdev->dev,
			"DIAG %s hop%u va=0x%llx BO[%u]+0x%llx: %08x %08x %08x %08x %08x %08x %08x %08x\n"
			"DIAG %s hop%u payload: %08x %08x %08x %08x %08x %08x %08x %08x\n",
			tag, hop, jc, i, off,
			le32_to_cpu(w[0]), le32_to_cpu(w[1]),
			le32_to_cpu(w[2]), le32_to_cpu(w[3]),
			le32_to_cpu(w[4]), le32_to_cpu(w[5]),
			le32_to_cpu(w[6]), le32_to_cpu(w[7]),
			tag, hop,
			le32_to_cpu(w[8]), le32_to_cpu(w[9]),
			le32_to_cpu(w[10]), le32_to_cpu(w[11]),
			le32_to_cpu(w[12]), le32_to_cpu(w[13]),
			le32_to_cpu(w[14]), le32_to_cpu(w[15]));

		next = (u64)le32_to_cpu(w[6]) |
		       ((u64)le32_to_cpu(w[7]) << 32);
		drm_gem_vunmap_unlocked(&bo->base.base, &map);
		if (next == jc)
			return;
		jc = next;
	}
}

static int panfrost_job_hw_submit(struct panfrost_job *job, int js)
{
	struct panfrost_device *pfdev = job->pfdev;
	unsigned int subslot;
	u32 cfg;
	u64 jc_head = job->jc;
	int ret;
	unsigned int i;

	panfrost_devfreq_record_busy(&pfdev->pfdevfreq);

	/*
	 * The job chain must live inside one of the buffers the job maps.  A jc
	 * that is covered by nothing makes the GPU fault as soon as it fetches
	 * the chain (head == tail, DATA_INVALID_FAULT) and, on this CV200
	 * integration, the address space then sticks until the whole GPU is
	 * reset.  Refuse the job instead of wedging the device, and say which
	 * address was not covered so the caller can be identified.
	 */
	for (i = 0; i < job->bo_count; i++) {
		struct panfrost_gem_mapping *m = job->mappings[i];
		u64 start, end;

		if (!m)
			continue;
		start = (u64)m->mmnode.start << PAGE_SHIFT;
		end = start + ((u64)m->mmnode.size << PAGE_SHIFT);
		if (jc_head >= start && jc_head < end)
			break;
	}

	if (i == job->bo_count) {
		dev_err(pfdev->dev,
			"job chain 0x%llx is outside the %u buffers of this job (js=%d), refusing to submit\n",
			jc_head, job->bo_count, js);
		panfrost_devfreq_record_idle(&pfdev->pfdevfreq);
		return -EINVAL;
	}

	ret = pm_runtime_get_sync(pfdev->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(pfdev->dev);
		panfrost_devfreq_record_idle(&pfdev->pfdevfreq);
		return ret;
	}

	if (WARN_ON(job_read(pfdev, JS_COMMAND_NEXT(js)))) {
		pm_runtime_mark_last_busy(pfdev->dev);
		pm_runtime_put_autosuspend(pfdev->dev);
		panfrost_devfreq_record_idle(&pfdev->pfdevfreq);
		return -EBUSY;
	}

	cfg = panfrost_mmu_as_get(pfdev, job->mmu);

	/*
	 * Diagnostic: the userspace must tell the GPU where the tiler heap is.
	 * Print the address the kernel assigned to the heap BO next to the
	 * contents of the job chain, so a heap address of zero in the chain can
	 * be attributed to the kernel or to the client.
	 */
	{
		static unsigned int diag_count;
		unsigned int i;

		for (i = 0; i < job->bo_count; i++) {
			struct panfrost_gem_object *bo =
				to_panfrost_bo(job->bos[i]);

			if (!bo->is_heap)
				continue;
			dev_info(pfdev->dev,
				 "DIAG heap job: js=%d jc=0x%llx heap_va=0x%llx heap_size=%zu map_active=%d\n",
				 js, jc_head,
				 job->mappings[i] ?
					(unsigned long long)(job->mappings[i]->mmnode.start << PAGE_SHIFT) : 0ULL,
				 bo->base.base.size,
				 job->mappings[i] ? job->mappings[i]->active : -1);
			break;
		}

		if (diag_count < 400) {
			dev_info(pfdev->dev, "DIAG submit js=%d jc=0x%llx comm=%s\n",
				 js, jc_head, current->comm);
			panfrost_diag_dump_va(pfdev, job, jc_head, "chain-submit");
			panfrost_diag_check_chain(pfdev, job, jc_head);
			if (diag_count < 120)
				panfrost_diag_walk_chain(pfdev, job, jc_head,
							 "submit");
			diag_count++;
		}
	}

	job_write(pfdev, JS_HEAD_NEXT_LO(js), lower_32_bits(jc_head));
	job_write(pfdev, JS_HEAD_NEXT_HI(js), upper_32_bits(jc_head));

	panfrost_job_write_affinity(pfdev, job->requirements, js);

	/* start MMU, medium priority, cache clean/flush on end, clean/flush on
	 * start */
	cfg |= JS_CONFIG_THREAD_PRI(8) |
		JS_CONFIG_START_FLUSH_CLEAN_INVALIDATE |
		JS_CONFIG_END_FLUSH_CLEAN_INVALIDATE |
		panfrost_get_job_chain_flag(job);

	if (panfrost_has_hw_feature(pfdev, HW_FEATURE_FLUSH_REDUCTION))
		cfg |= JS_CONFIG_ENABLE_FLUSH_REDUCTION;

	if (panfrost_has_hw_issue(pfdev, HW_ISSUE_10649))
		cfg |= JS_CONFIG_START_MMU;

	job_write(pfdev, JS_CONFIG_NEXT(js), cfg);

	{
		static unsigned int diag_cfg;

		if (diag_cfg++ < 300)
			dev_info(pfdev->dev,
				 "DIAG JSCFG js=%d cfg=0x%x reqs=0x%x jc=0x%llx pid=%d comm=%s\n",
				 js, cfg, job->requirements, jc_head,
				 job->diag_pid, job->diag_comm);
	}

	if (panfrost_has_hw_feature(pfdev, HW_FEATURE_FLUSH_REDUCTION))
		job_write(pfdev, JS_FLUSH_ID_NEXT(js), job->flush_id);

	/* GO ! */

	spin_lock(&pfdev->js->job_lock);
	subslot = panfrost_enqueue_job(pfdev, js, job);
	/* Don't queue the job if a reset is in progress */
	if (!atomic_read(&pfdev->reset.pending)) {
		if (pfdev->profile_mode) {
			panfrost_cycle_counter_get(pfdev);
			job->is_profiled = true;
			job->start_time = ktime_get();
			job->start_cycles = panfrost_cycle_counter_read(pfdev);
		}

		job_write(pfdev, JS_COMMAND_NEXT(js), JS_COMMAND_START);
		dev_dbg(pfdev->dev,
			"JS: Submitting atom %p to js[%d][%d] with head=0x%llx AS %d",
			job, js, subslot, jc_head, cfg & 0xf);
	}
	spin_unlock(&pfdev->js->job_lock);

	return 0;
}

static int panfrost_acquire_object_fences(struct drm_gem_object **bos,
					  int bo_count,
					  struct drm_sched_job *job)
{
	int i, ret;

	for (i = 0; i < bo_count; i++) {
		ret = dma_resv_reserve_fences(bos[i]->resv, 1);
		if (ret)
			return ret;

		/* panfrost always uses write mode in its current uapi */
		ret = drm_sched_job_add_implicit_dependencies(job, bos[i],
							      true);
		if (ret)
			return ret;
	}

	return 0;
}

static void panfrost_attach_object_fences(struct drm_gem_object **bos,
					  int bo_count,
					  struct dma_fence *fence)
{
	int i;

	for (i = 0; i < bo_count; i++)
		dma_resv_add_fence(bos[i]->resv, fence, DMA_RESV_USAGE_WRITE);
}

int panfrost_job_push(struct panfrost_job *job)
{
	struct panfrost_device *pfdev = job->pfdev;
	struct ww_acquire_ctx acquire_ctx;
	int ret = 0;

	ret = drm_gem_lock_reservations(job->bos, job->bo_count,
					    &acquire_ctx);
	if (ret)
		return ret;

	mutex_lock(&pfdev->sched_lock);
	drm_sched_job_arm(&job->base);

	job->render_done_fence = dma_fence_get(&job->base.s_fence->finished);

	ret = panfrost_acquire_object_fences(job->bos, job->bo_count,
					     &job->base);
	if (ret) {
		mutex_unlock(&pfdev->sched_lock);
		goto unlock;
	}

	kref_get(&job->refcount); /* put by scheduler job completion */

	drm_sched_entity_push_job(&job->base);

	mutex_unlock(&pfdev->sched_lock);

	panfrost_attach_object_fences(job->bos, job->bo_count,
				      job->render_done_fence);

unlock:
	drm_gem_unlock_reservations(job->bos, job->bo_count, &acquire_ctx);

	return ret;
}

static void panfrost_job_cleanup(struct kref *ref)
{
	struct panfrost_job *job = container_of(ref, struct panfrost_job,
						refcount);
	unsigned int i;

	dma_fence_put(job->done_fence);
	dma_fence_put(job->render_done_fence);

	if (job->mappings) {
		for (i = 0; i < job->bo_count; i++) {
			if (!job->mappings[i])
				break;

			atomic_dec(&job->mappings[i]->obj->gpu_usecount);
			panfrost_gem_mapping_put(job->mappings[i]);
		}
		kvfree(job->mappings);
	}

	if (job->bos) {
		for (i = 0; i < job->bo_count; i++)
			drm_gem_object_put(job->bos[i]);

		kvfree(job->bos);
	}

	kfree(job);
}

void panfrost_job_put(struct panfrost_job *job)
{
	kref_put(&job->refcount, panfrost_job_cleanup);
}

static void panfrost_job_free(struct drm_sched_job *sched_job)
{
	struct panfrost_job *job = to_panfrost_job(sched_job);

	drm_sched_job_cleanup(sched_job);

	panfrost_job_put(job);
}

static struct dma_fence *panfrost_job_run(struct drm_sched_job *sched_job)
{
	struct panfrost_job *job = to_panfrost_job(sched_job);
	struct panfrost_device *pfdev = job->pfdev;
	int slot = panfrost_job_get_slot(job);
	struct dma_fence *fence = NULL;
	int ret;

	if (unlikely(job->base.s_fence->finished.error))
		return NULL;

	/* Nothing to execute: can happen if the job has finished while
	 * we were resetting the GPU.
	 */
	if (!job->jc)
		return NULL;

	fence = panfrost_fence_create(pfdev, slot);
	if (IS_ERR(fence))
		return fence;

	if (job->done_fence)
		dma_fence_put(job->done_fence);
	job->done_fence = dma_fence_get(fence);

	ret = panfrost_job_hw_submit(job, slot);
	if (ret) {
		/* Keep scheduler progress and the userspace fence intact if the
		 * device cannot be resumed or the slot is unexpectedly occupied. */
		dma_fence_set_error(fence, ret);
		dma_fence_signal(fence);
	}

	return fence;
}

void panfrost_job_enable_interrupts(struct panfrost_device *pfdev)
{
	int j;
	u32 irq_mask = 0;

	clear_bit(PANFROST_COMP_BIT_JOB, pfdev->is_suspended);

	for (j = 0; j < NUM_JOB_SLOTS; j++) {
		irq_mask |= MK_JS_MASK(j);
	}

	job_write(pfdev, JOB_INT_CLEAR, irq_mask);
	job_write(pfdev, JOB_INT_MASK, irq_mask);
}

void panfrost_job_suspend_irq(struct panfrost_device *pfdev)
{
	set_bit(PANFROST_COMP_BIT_JOB, pfdev->is_suspended);

	job_write(pfdev, JOB_INT_MASK, 0);
	synchronize_irq(pfdev->js->irq);
}

static void panfrost_job_handle_err(struct panfrost_device *pfdev,
				    struct panfrost_job *job,
				    unsigned int js)
{
	u32 js_status = job_read(pfdev, JS_STATUS(js));
	const char *exception_name = panfrost_exception_name(js_status);
	bool signal_fence = true;

	if (!panfrost_exception_is_fault(js_status)) {
		dev_dbg(pfdev->dev, "js event, js=%d, status=%s, head=0x%x, tail=0x%x",
			js, exception_name,
			job_read(pfdev, JS_HEAD_LO(js)),
			job_read(pfdev, JS_TAIL_LO(js)));
	} else {
		dev_err(pfdev->dev, "js fault, js=%d, status=%s, head=0x%x, tail=0x%x",
			js, exception_name,
			job_read(pfdev, JS_HEAD_LO(js)),
			job_read(pfdev, JS_TAIL_LO(js)));
	}

	/*
	 * Diagnostic: capture the chain the GPU was executing before the reset
	 * work runs, while the job buffers are still mapped.
	 */
	if (panfrost_exception_is_fault(js_status)) {
		static unsigned int fault_diag;

		if (fault_diag++ < 400) {
			u64 head = (u64)job_read(pfdev, JS_HEAD_LO(js)) |
				  ((u64)job_read(pfdev, JS_HEAD_HI(js)) << 32);

			dev_err(pfdev->dev,
				"DIAG fault js=%d jc=0x%llx head=0x%llx tail=0x%llx nbos=%u as=%d pid=%d comm=%s\n",
				js, job->jc, head,
				(u64)job_read(pfdev, JS_TAIL_LO(js)) |
				((u64)job_read(pfdev, JS_TAIL_HI(js)) << 32),
				job->bo_count, job->mmu ? job->mmu->as : -1,
				job->diag_pid, job->diag_comm);
			panfrost_diag_dump_va(pfdev, job, job->jc, "chain-fault");
			panfrost_diag_dump_va(pfdev, job, head, "head-fault");
			panfrost_diag_check_chain(pfdev, job, job->jc);
			if (fault_diag <= 120)
				panfrost_diag_walk_chain(pfdev, job, job->jc,
							 "fault");
		}
	}

	if (js_status == DRM_PANFROST_EXCEPTION_STOPPED) {
		/* Update the job head so we can resume */
		job->jc = job_read(pfdev, JS_TAIL_LO(js)) |
			  ((u64)job_read(pfdev, JS_TAIL_HI(js)) << 32);

		/* The job will be resumed, don't signal the fence */
		signal_fence = false;
	} else if (js_status == DRM_PANFROST_EXCEPTION_TERMINATED) {
		/* Job has been hard-stopped, flag it as canceled */
		dma_fence_set_error(job->done_fence, -ECANCELED);
		job->jc = 0;
	} else if (panfrost_exception_is_fault(js_status)) {
		/* We might want to provide finer-grained error code based on
		 * the exception type, but unconditionally setting to EINVAL
		 * is good enough for now.
		 */
		dma_fence_set_error(job->done_fence, -EINVAL);
		job->jc = 0;
	}

	panfrost_mmu_as_put(pfdev, job->mmu);
	panfrost_devfreq_record_idle(&pfdev->pfdevfreq);

	if (signal_fence)
		dma_fence_signal_locked(job->done_fence);

	pm_runtime_put_autosuspend(pfdev->dev);

	if (panfrost_exception_needs_reset(pfdev, js_status)) {
		atomic_set(&pfdev->reset.pending, 1);
		drm_sched_fault(&pfdev->js->queue[js].sched);
	}
}

static void panfrost_job_handle_done(struct panfrost_device *pfdev,
				     struct panfrost_job *job)
{
	/* Set ->jc to 0 to avoid re-submitting an already finished job (can
	 * happen when we receive the DONE interrupt while doing a GPU reset).
	 */
	job->jc = 0;
	panfrost_mmu_as_put(pfdev, job->mmu);
	panfrost_devfreq_record_idle(&pfdev->pfdevfreq);

	dma_fence_signal_locked(job->done_fence);
	pm_runtime_put_autosuspend(pfdev->dev);
}

static void panfrost_job_handle_irq(struct panfrost_device *pfdev, u32 status)
{
	struct panfrost_job *done[NUM_JOB_SLOTS][2] = {};
	struct panfrost_job *failed[NUM_JOB_SLOTS] = {};
	u32 js_state = 0, js_events = 0;
	unsigned int i, j;

	/* First we collect all failed/done jobs. */
	while (status) {
		u32 js_state_mask = 0;

		for (j = 0; j < NUM_JOB_SLOTS; j++) {
			if (status & MK_JS_MASK(j))
				js_state_mask |= MK_JS_MASK(j);

			if (status & JOB_INT_MASK_DONE(j)) {
				if (done[j][0])
					done[j][1] = panfrost_dequeue_job(pfdev, j);
				else
					done[j][0] = panfrost_dequeue_job(pfdev, j);
			}

			if (status & JOB_INT_MASK_ERR(j)) {
				/* Cancel the next submission. Will be submitted
				 * after we're done handling this failure if
				 * there's no reset pending.
				 */
				job_write(pfdev, JS_COMMAND_NEXT(j), JS_COMMAND_NOP);
				failed[j] = panfrost_dequeue_job(pfdev, j);
			}
		}

		/* JS_STATE is sampled when JOB_INT_CLEAR is written.
		 * For each BIT(slot) or BIT(slot + 16) bit written to
		 * JOB_INT_CLEAR, the corresponding bits in JS_STATE
		 * (BIT(slot) and BIT(slot + 16)) are updated, but this
		 * is racy. If we only have one job done at the time we
		 * read JOB_INT_RAWSTAT but the second job fails before we
		 * clear the status, we end up with a status containing
		 * only the DONE bit and consider both jobs as DONE since
		 * JS_STATE reports both NEXT and CURRENT as inactive.
		 * To prevent that, let's repeat this clear+read steps
		 * until status is 0.
		 */
		job_write(pfdev, JOB_INT_CLEAR, status);
		js_state &= ~js_state_mask;
		js_state |= job_read(pfdev, JOB_INT_JS_STATE) & js_state_mask;
		js_events |= status;
		status = job_read(pfdev, JOB_INT_RAWSTAT);
	}

	/* Then we handle the dequeued jobs. */
	for (j = 0; j < NUM_JOB_SLOTS; j++) {
		if (!(js_events & MK_JS_MASK(j)))
			continue;

		if (failed[j]) {
			panfrost_job_handle_err(pfdev, failed[j], j);
		} else if (pfdev->jobs[j][0] && !(js_state & MK_JS_MASK(j))) {
			/* When the current job doesn't fail, the JM dequeues
			 * the next job without waiting for an ACK, this means
			 * we can have 2 jobs dequeued and only catch the
			 * interrupt when the second one is done. If both slots
			 * are inactive, but one job remains in pfdev->jobs[j],
			 * consider it done. Of course that doesn't apply if a
			 * failure happened since we cancelled execution of the
			 * job in _NEXT (see above).
			 */
			if (WARN_ON(!done[j][0]))
				done[j][0] = panfrost_dequeue_job(pfdev, j);
			else
				done[j][1] = panfrost_dequeue_job(pfdev, j);
		}

		for (i = 0; i < ARRAY_SIZE(done[0]) && done[j][i]; i++)
			panfrost_job_handle_done(pfdev, done[j][i]);
	}

	/* And finally we requeue jobs that were waiting in the second slot
	 * and have been stopped if we detected a failure on the first slot.
	 */
	for (j = 0; j < NUM_JOB_SLOTS; j++) {
		if (!(js_events & MK_JS_MASK(j)))
			continue;

		if (!failed[j] || !pfdev->jobs[j][0])
			continue;

		if (pfdev->jobs[j][0]->jc == 0) {
			/* The job was cancelled, signal the fence now */
			struct panfrost_job *canceled = panfrost_dequeue_job(pfdev, j);

			dma_fence_set_error(canceled->done_fence, -ECANCELED);
			panfrost_job_handle_done(pfdev, canceled);
		} else if (!atomic_read(&pfdev->reset.pending)) {
			/* Requeue the job we removed if no reset is pending */
			job_write(pfdev, JS_COMMAND_NEXT(j), JS_COMMAND_START);
		}
	}
}

static void panfrost_job_handle_irqs(struct panfrost_device *pfdev)
{
	u32 status = job_read(pfdev, JOB_INT_RAWSTAT);

	while (status) {
		pm_runtime_mark_last_busy(pfdev->dev);

		spin_lock(&pfdev->js->job_lock);
		panfrost_job_handle_irq(pfdev, status);
		spin_unlock(&pfdev->js->job_lock);
		status = job_read(pfdev, JOB_INT_RAWSTAT);
	}
}

static u32 panfrost_active_slots(struct panfrost_device *pfdev,
				 u32 *js_state_mask, u32 js_state)
{
	u32 rawstat;

	if (!(js_state & *js_state_mask))
		return 0;

	rawstat = job_read(pfdev, JOB_INT_RAWSTAT);
	if (rawstat) {
		unsigned int i;

		for (i = 0; i < NUM_JOB_SLOTS; i++) {
			if (rawstat & MK_JS_MASK(i))
				*js_state_mask &= ~MK_JS_MASK(i);
		}
	}

	return js_state & *js_state_mask;
}

static int
panfrost_reset(struct panfrost_device *pfdev,
	       struct drm_sched_job *bad)
{
	u32 js_state, js_state_mask = 0xffffffff;
	unsigned int i, j;
	bool cookie;
	int ret;

	if (!atomic_read(&pfdev->reset.pending))
		return 0;

	/* Stop the schedulers.
	 *
	 * FIXME: We temporarily get out of the dma_fence_signalling section
	 * because the cleanup path generate lockdep splats when taking locks
	 * to release job resources. We should rework the code to follow this
	 * pattern:
	 *
	 *	try_lock
	 *	if (locked)
	 *		release
	 *	else
	 *		schedule_work_to_release_later
	 */
	for (i = 0; i < NUM_JOB_SLOTS; i++)
		drm_sched_stop(&pfdev->js->queue[i].sched, bad);

	cookie = dma_fence_begin_signalling();

	if (bad)
		drm_sched_increase_karma(bad);

	/* Mask job interrupts and synchronize to make sure we won't be
	 * interrupted during our reset.
	 */
	job_write(pfdev, JOB_INT_MASK, 0);
	synchronize_irq(pfdev->js->irq);

	for (i = 0; i < NUM_JOB_SLOTS; i++) {
		/* Cancel the next job and soft-stop the running job. */
		job_write(pfdev, JS_COMMAND_NEXT(i), JS_COMMAND_NOP);
		job_write(pfdev, JS_COMMAND(i), JS_COMMAND_SOFT_STOP);
	}

	/* Wait at most 10ms for soft-stops to complete */
	ret = readl_poll_timeout(pfdev->iomem + JOB_INT_JS_STATE, js_state,
				 !panfrost_active_slots(pfdev, &js_state_mask, js_state),
				 10, 10000);

	if (ret)
		dev_err(pfdev->dev, "Soft-stop failed\n");

	/* Handle the remaining interrupts before we reset. */
	panfrost_job_handle_irqs(pfdev);

	/* Remaining interrupts have been handled, but we might still have
	 * stuck jobs. Let's make sure the PM counters stay balanced by
	 * manually calling pm_runtime_put_noidle() and
	 * panfrost_devfreq_record_idle() for each stuck job.
	 * Let's also make sure the cycle counting register's refcnt is
	 * kept balanced to prevent it from running forever
	 */
	spin_lock(&pfdev->js->job_lock);
	for (i = 0; i < NUM_JOB_SLOTS; i++) {
		for (j = 0; j < ARRAY_SIZE(pfdev->jobs[0]) && pfdev->jobs[i][j]; j++) {
			if (pfdev->jobs[i][j]->is_profiled)
				panfrost_cycle_counter_put(pfdev->jobs[i][j]->pfdev);
			pm_runtime_put_noidle(pfdev->dev);
			panfrost_devfreq_record_idle(&pfdev->pfdevfreq);
		}
	}
	memset(pfdev->jobs, 0, sizeof(pfdev->jobs));
	spin_unlock(&pfdev->js->job_lock);

	/* Proceed with reset now. */
	ret = panfrost_device_reset(pfdev);
	if (ret) {
		dev_err(pfdev->dev,
			"GPU reset failed: %d; leaving schedulers stopped\n", ret);
		dma_fence_end_signalling(cookie);
		return ret;
	}

	/* panfrost_device_reset() unmasks job interrupts, but we want to
	 * keep them masked a bit longer.
	 */
	job_write(pfdev, JOB_INT_MASK, 0);

	/* GPU has been reset, we can clear the reset pending bit. */
	atomic_set(&pfdev->reset.pending, 0);

	/* Now resubmit jobs that were previously queued but didn't have a
	 * chance to finish.
	 * FIXME: We temporarily get out of the DMA fence signalling section
	 * while resubmitting jobs because the job submission logic will
	 * allocate memory with the GFP_KERNEL flag which can trigger memory
	 * reclaim and exposes a lock ordering issue.
	 */
	dma_fence_end_signalling(cookie);
	for (i = 0; i < NUM_JOB_SLOTS; i++)
		drm_sched_resubmit_jobs(&pfdev->js->queue[i].sched);
	cookie = dma_fence_begin_signalling();

	/* Restart the schedulers */
	for (i = 0; i < NUM_JOB_SLOTS; i++)
		drm_sched_start(&pfdev->js->queue[i].sched);

	/* Re-enable job interrupts now that everything has been restarted. */
	job_write(pfdev, JOB_INT_MASK,
		  GENMASK(16 + NUM_JOB_SLOTS - 1, 16) |
		  GENMASK(NUM_JOB_SLOTS - 1, 0));

	dma_fence_end_signalling(cookie);
	return 0;
}

static enum drm_gpu_sched_stat panfrost_job_timedout(struct drm_sched_job
						     *sched_job)
{
	struct panfrost_job *job = to_panfrost_job(sched_job);
	struct panfrost_device *pfdev = job->pfdev;
	int js = panfrost_job_get_slot(job);

	/*
	 * If the GPU managed to complete this jobs fence, the timeout is
	 * spurious. Bail out.
	 */
	if (dma_fence_is_signaled(job->done_fence))
		return DRM_GPU_SCHED_STAT_NOMINAL;

	/*
	 * Panfrost IRQ handler may take a long time to process an interrupt
	 * if there is another IRQ handler hogging the processing.
	 * For example, the HDMI encoder driver might be stuck in the IRQ
	 * handler for a significant time in a case of bad cable connection.
	 * In order to catch such cases and not report spurious Panfrost
	 * job timeouts, synchronize the IRQ handler and re-check the fence
	 * status.
	 */
	synchronize_irq(pfdev->js->irq);

	if (dma_fence_is_signaled(job->done_fence)) {
		dev_warn(pfdev->dev, "unexpectedly high interrupt latency\n");
		return DRM_GPU_SCHED_STAT_NOMINAL;
	}

	dev_err(pfdev->dev, "gpu sched timeout, js=%d, config=0x%x, status=0x%x, head=0x%x, tail=0x%x, sched_job=%p",
		js,
		job_read(pfdev, JS_CONFIG(js)),
		job_read(pfdev, JS_STATUS(js)),
		job_read(pfdev, JS_HEAD_LO(js)),
		job_read(pfdev, JS_TAIL_LO(js)),
		sched_job);

	panfrost_core_dump(job);

	atomic_set(&pfdev->reset.pending, 1);
	if (panfrost_reset(pfdev, sched_job))
		return DRM_GPU_SCHED_STAT_ENODEV;

	return DRM_GPU_SCHED_STAT_NOMINAL;
}

static void panfrost_reset_work(struct work_struct *work)
{
	struct panfrost_device *pfdev;

	pfdev = container_of(work, struct panfrost_device, reset.work);
	if (panfrost_reset(pfdev, NULL))
		dev_err(pfdev->dev,
			"GPU reset work failed; scheduler remains stopped\n");
}

static const struct drm_sched_backend_ops panfrost_sched_ops = {
	.run_job = panfrost_job_run,
	.timedout_job = panfrost_job_timedout,
	.free_job = panfrost_job_free
};

static irqreturn_t panfrost_job_irq_handler_thread(int irq, void *data)
{
	struct panfrost_device *pfdev = data;

	panfrost_job_handle_irqs(pfdev);

	/* Enable interrupts only if we're not about to get suspended */
	if (!test_bit(PANFROST_COMP_BIT_JOB, pfdev->is_suspended))
		job_write(pfdev, JOB_INT_MASK,
			  GENMASK(16 + NUM_JOB_SLOTS - 1, 16) |
			  GENMASK(NUM_JOB_SLOTS - 1, 0));

	return IRQ_HANDLED;
}

static irqreturn_t panfrost_job_irq_handler(int irq, void *data)
{
	struct panfrost_device *pfdev = data;
	u32 status;

	if (test_bit(PANFROST_COMP_BIT_JOB, pfdev->is_suspended))
		return IRQ_NONE;

	status = job_read(pfdev, JOB_INT_STAT);
	if (!status)
		return IRQ_NONE;

	job_write(pfdev, JOB_INT_MASK, 0);
	return IRQ_WAKE_THREAD;
}

int panfrost_job_init(struct panfrost_device *pfdev)
{
	struct panfrost_job_slot *js;
	unsigned int nentries = 2;
	int ret, j;

	/* All GPUs have two entries per queue, but without jobchain
	 * disambiguation stopping the right job in the close path is tricky,
	 * so let's just advertise one entry in that case.
	 */
	if (!panfrost_has_hw_feature(pfdev, HW_FEATURE_JOBCHAIN_DISAMBIGUATION))
		nentries = 1;

	pfdev->js = js = devm_kzalloc(pfdev->dev, sizeof(*js), GFP_KERNEL);
	if (!js)
		return -ENOMEM;

	INIT_WORK(&pfdev->reset.work, panfrost_reset_work);
	spin_lock_init(&js->job_lock);

	js->irq = platform_get_irq_byname(to_platform_device(pfdev->dev), "job");
	if (js->irq < 0)
		return js->irq;

	ret = devm_request_threaded_irq(pfdev->dev, js->irq,
					panfrost_job_irq_handler,
					panfrost_job_irq_handler_thread,
					IRQF_SHARED, KBUILD_MODNAME "-job",
					pfdev);
	if (ret) {
		dev_err(pfdev->dev, "failed to request job irq");
		return ret;
	}

	pfdev->reset.wq = alloc_ordered_workqueue("panfrost-reset", 0);
	if (!pfdev->reset.wq)
		return -ENOMEM;

	for (j = 0; j < NUM_JOB_SLOTS; j++) {
		js->queue[j].fence_context = dma_fence_context_alloc(1);

		ret = drm_sched_init(&js->queue[j].sched,
				     &panfrost_sched_ops, NULL,
				     DRM_SCHED_PRIORITY_COUNT,
				     nentries, 0,
				     msecs_to_jiffies(JOB_TIMEOUT_MS),
				     pfdev->reset.wq,
				     NULL, "pan_js", pfdev->dev);
		if (ret) {
			dev_err(pfdev->dev, "Failed to create scheduler: %d.", ret);
			goto err_sched;
		}
	}

	panfrost_job_enable_interrupts(pfdev);

	dev_err(pfdev->dev,
		"DIAG panfrost_job_init marker: chain walker build loaded\n");

	return 0;

err_sched:
	for (j--; j >= 0; j--)
		drm_sched_fini(&js->queue[j].sched);

	destroy_workqueue(pfdev->reset.wq);
	return ret;
}

void panfrost_job_fini(struct panfrost_device *pfdev)
{
	struct panfrost_job_slot *js = pfdev->js;
	int j;

	job_write(pfdev, JOB_INT_MASK, 0);

	for (j = 0; j < NUM_JOB_SLOTS; j++) {
		drm_sched_fini(&js->queue[j].sched);
	}

	cancel_work_sync(&pfdev->reset.work);
	destroy_workqueue(pfdev->reset.wq);
}

int panfrost_job_open(struct panfrost_file_priv *panfrost_priv)
{
	struct panfrost_device *pfdev = panfrost_priv->pfdev;
	struct panfrost_job_slot *js = pfdev->js;
	struct drm_gpu_scheduler *sched;
	int ret, i;

	for (i = 0; i < NUM_JOB_SLOTS; i++) {
		sched = &js->queue[i].sched;
		ret = drm_sched_entity_init(&panfrost_priv->sched_entity[i],
					    DRM_SCHED_PRIORITY_NORMAL, &sched,
					    1, NULL);
		if (WARN_ON(ret))
			return ret;
	}
	return 0;
}

void panfrost_job_close(struct panfrost_file_priv *panfrost_priv)
{
	struct panfrost_device *pfdev = panfrost_priv->pfdev;
	int i;

	for (i = 0; i < NUM_JOB_SLOTS; i++)
		drm_sched_entity_destroy(&panfrost_priv->sched_entity[i]);

	/* Kill in-flight jobs */
	spin_lock(&pfdev->js->job_lock);
	for (i = 0; i < NUM_JOB_SLOTS; i++) {
		struct drm_sched_entity *entity = &panfrost_priv->sched_entity[i];
		int j;

		for (j = ARRAY_SIZE(pfdev->jobs[0]) - 1; j >= 0; j--) {
			struct panfrost_job *job = pfdev->jobs[i][j];
			u32 cmd;

			if (!job || job->base.entity != entity)
				continue;

			if (j == 1) {
				/* Try to cancel the job before it starts */
				job_write(pfdev, JS_COMMAND_NEXT(i), JS_COMMAND_NOP);
				/* Reset the job head so it doesn't get restarted if
				 * the job in the first slot failed.
				 */
				job->jc = 0;
			}

			if (panfrost_has_hw_feature(pfdev, HW_FEATURE_JOBCHAIN_DISAMBIGUATION)) {
				cmd = panfrost_get_job_chain_flag(job) ?
				      JS_COMMAND_HARD_STOP_1 :
				      JS_COMMAND_HARD_STOP_0;
			} else {
				cmd = JS_COMMAND_HARD_STOP;
			}

			job_write(pfdev, JS_COMMAND(i), cmd);

			/* Jobs can outlive their file context */
			job->engine_usage = NULL;
		}
	}
	spin_unlock(&pfdev->js->job_lock);
}

int panfrost_job_is_idle(struct panfrost_device *pfdev)
{
	struct panfrost_job_slot *js = pfdev->js;
	int i;

	for (i = 0; i < NUM_JOB_SLOTS; i++) {
		/* If there are any jobs in the HW queue, we're not idle */
		if (atomic_read(&js->queue[i].sched.credit_count))
			return false;
	}

	return true;
}
