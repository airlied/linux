

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
		struct i915_ttm_bo *bo = ttm_to_i915_bo(e->tv.bo);

		i915_ttm_bo_unref(&bo);
	}

	call_rcu(&list->rhead, i915_ttm_bo_list_free_rcu);
}


int i915_ttm_bo_list_create(struct drm_i915_private *i915,
			    struct drm_file *file,
			    struct drm_i915_gem_exec_object2 *exec,
			    unsigned num_entries,
			    struct i915_ttm_bo_list **result)
{
	unsigned last_entry;
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
		struct drm_gem_object *gobj;
		struct i915_ttm_bo *bo;

		gobj = drm_gem_object_lookup(file, exec[i].handle);
		if (!gobj) {
			r = -ENOENT;
			goto error_free;
		}

		bo = i915_ttm_bo_ref(ttm_gem_to_i915_bo(gobj));
		drm_gem_object_put(gobj);

		entry = &array[last_entry++];
		
		entry->tv.bo = &bo->tbo;

		total_size += i915_ttm_bo_size(bo);
	}
	list->num_entries = num_entries;
	*result = 0;
	return 0;
error_free:
	for (i = 0; i < last_entry; ++i) {
		struct i915_ttm_bo *bo = ttm_to_i915_bo(array[i].tv.bo);
		i915_ttm_bo_unref(&bo);
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
		struct i915_ttm_bo *bo = ttm_to_i915_bo(e->tv.bo);

		list_add_tail(&e->tv.head, validated);
	}
}

void i915_ttm_bo_list_put(struct i915_ttm_bo_list *list)
{
	kref_put(&list->refcount, i915_ttm_bo_list_free);
}
