

#include "i915_drv.h"
#include "ttm/i915_ttm.h"

static void i915_ttm_bo_list_free_rcu(struct rcu_head *rcu)
{
	struct i915_ttm_bo_list *list = container_of(rcu, struct i915_ttm_bo_list,
						   rhead);

	kvfree(list);
}

static void i915_ttm_bo_list_free(struct kref *ref)
{
	struct i915_ttm_bo_list *list = container_of(ref, struct i915_ttm_bo_list,
						   refcount);
	struct i915_ttm_bo_list_entry *e;

	i915_ttm_bo_list_for_each_entry(e, list) {
		struct drm_i915_gem_object *obj = ttm_to_i915_gem(e->tv.bo);
		i915_gem_object_put(obj);
	}

	call_rcu(&list->rhead, i915_ttm_bo_list_free_rcu);
}

static u64 eb_pin_flags(const struct drm_i915_gem_exec_object2 *entry,
			unsigned int exec_flags)
{
	u64 pin_flags = 0;

	if (exec_flags & EXEC_OBJECT_NEEDS_GTT)
		pin_flags |= PIN_GLOBAL;

	/*
	 * Wa32bitGeneralStateOffset & Wa32bitInstructionBaseOffset,
	 * limit address to the first 4GBs for unflagged objects.
	 */
	if (!(exec_flags & EXEC_OBJECT_SUPPORTS_48B_ADDRESS))
		pin_flags |= PIN_ZONE_4G;

//	if (exec_flags & __EXEC_OBJECT_NEEDS_MAP)
//		pin_flags |= PIN_MAPPABLE;

	if (exec_flags & EXEC_OBJECT_PINNED)
		pin_flags |= entry->offset | PIN_OFFSET_FIXED;
//	else if (exec_flags & __EXEC_OBJECT_NEEDS_BIAS)
//		pin_flags |= BATCH_OFFSET_BIAS | PIN_OFFSET_BIAS;

	return pin_flags;
}



int i915_ttm_bo_list_create(struct drm_i915_private *i915,
			    struct drm_file *file,
			    struct drm_i915_gem_exec_object2 *exec,
			    unsigned num_entries,
			    struct i915_ttm_bo_list **result)
{
	unsigned last_entry = 0;
	struct i915_ttm_bo_list_entry *array;
	struct i915_ttm_bo_list *list;
	size_t size;
	uint64_t total_size = 0;
	unsigned i;
	int r;
	if (num_entries > (SIZE_MAX - sizeof(struct i915_ttm_bo_list))
				/ sizeof(struct i915_ttm_bo_list_entry))
		return -EINVAL;

	size = sizeof(struct i915_ttm_bo_list);
	size += num_entries * sizeof(struct i915_ttm_bo_list_entry);
	list = kvmalloc(size, GFP_KERNEL);
	if (!list)
		return -ENOMEM;

	kref_init(&list->refcount);

	array = i915_ttm_bo_list_array_entry(list, 0);
	memset(array, 0, num_entries * sizeof(struct i915_ttm_bo_list_entry));

	for (i = 0; i < num_entries; ++i) {
		struct i915_ttm_bo_list_entry *entry;
		struct drm_i915_gem_object *obj;

		obj = i915_gem_object_lookup(file, exec[i].handle);
		if (!obj) {
			r = -ENOENT;
			goto error_free;
		}

		entry = &array[last_entry++];

		entry->user_flags = exec[i].flags;
		entry->tv.bo = &obj->base;

		exec[i].offset = gen8_noncanonical_addr(exec[i].offset);

		entry->pin_flags = eb_pin_flags(&exec[i], EXEC_OBJECT_PINNED | EXEC_OBJECT_SUPPORTS_48B_ADDRESS);
		total_size += i915_gem_object_size(obj);
	}
	list->num_entries = num_entries;
	*result = list;
	return 0;
error_free:
	for (i = 0; i < last_entry; ++i) {
		struct drm_i915_gem_object *obj = ttm_to_i915_gem(array[i].tv.bo);
		i915_gem_object_put(obj);
	}
	kvfree(list);
	return r;
}

void i915_ttm_bo_list_get_list(struct i915_ttm_bo_list *list,
			       struct list_head *validated)
{
	/* This is based on the bucket sort with O(n) time complexity.
	 * An item with priority "i" is added to bucket[i]. The lists are then
	 * concatenated in descending order.
	 */
	struct i915_ttm_bo_list_entry *e;
	unsigned i;

	i915_ttm_bo_list_for_each_entry(e, list) {
		list_add_tail(&e->tv.head, validated);
	}
}

void i915_ttm_bo_list_put(struct i915_ttm_bo_list *list)
{
	kref_put(&list->refcount, i915_ttm_bo_list_free);
}
