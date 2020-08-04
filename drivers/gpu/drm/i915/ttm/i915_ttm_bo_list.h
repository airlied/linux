// SPDX-License-Identifier: MIT
/*
 * Copyright @ 2020 Red Hat.
 */
#ifndef __I915_TTM_BO_LIST_H__
#define __I915_TTM_BO_LIST_H__

#include "drm/ttm/ttm_execbuf_util.h"
struct drm_i915_private;
struct drm_file;
struct drm_i915_gem_exec_object2;

struct i915_ttm_bo_list_entry {
	struct ttm_validate_buffer tv;
	struct i915_vma *vma;
	u32 user_flags;
	u64 pin_flags;
	bool vma_pinned;
};

struct i915_ttm_bo_list {
	struct rcu_head rhead;
	struct kref refcount;
	unsigned num_entries;	
};

static inline struct i915_ttm_bo_list_entry *
i915_ttm_bo_list_array_entry(struct i915_ttm_bo_list *list, unsigned index)
{
	struct i915_ttm_bo_list_entry *array = (void *)&list[1];

	return &array[index];
}

#define i915_ttm_bo_list_for_each_entry(e, list) \
	for (e = i915_ttm_bo_list_array_entry(list, 0); \
	     e != i915_ttm_bo_list_array_entry(list, (list)->num_entries); \
	     ++e)

#define i915_ttm_bo_list_for_each_userptr_entry(e, list) \
	for (e = i915_ttm_bo_list_array_entry(list, (list)->first_userptr); \
	     e != i915_ttm_bo_list_array_entry(list, (list)->num_entries); \
	     ++e)



int i915_ttm_bo_list_create(struct drm_i915_private *i915,
			    struct drm_file *filp,
			    struct drm_i915_gem_exec_object2 *exec,
			    unsigned num_entries,
			    struct i915_ttm_bo_list **result);

void i915_ttm_bo_list_put(struct i915_ttm_bo_list *list);
void i915_ttm_bo_list_get_list(struct i915_ttm_bo_list *list,
			       struct list_head *validated);

#endif
