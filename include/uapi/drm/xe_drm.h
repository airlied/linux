/*
 * Copyright 2021 Intel Corporation. All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sub license, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * IN NO EVENT SHALL TUNGSTEN GRAPHICS AND/OR ITS SUPPLIERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#ifndef _UAPI_XE_DRM_H_
#define _UAPI_XE_DRM_H_

#include "drm.h"

#if defined(__cplusplus)
extern "C" {
#endif

/* Please note that modifications to all structs defined here are
 * subject to backwards-compatibility constraints.
 */

/**
 * struct i915_user_extension - Base class for defining a chain of extensions
 *
 * Many interfaces need to grow over time. In most cases we can simply
 * extend the struct and have userspace pass in more data. Another option,
 * as demonstrated by Vulkan's approach to providing extensions for forward
 * and backward compatibility, is to use a list of optional structs to
 * provide those extra details.
 *
 * The key advantage to using an extension chain is that it allows us to
 * redefine the interface more easily than an ever growing struct of
 * increasing complexity, and for large parts of that interface to be
 * entirely optional. The downside is more pointer chasing; chasing across
 * the __user boundary with pointers encapsulated inside u64.
 *
 * Example chaining:
 *
 * .. code-block:: C
 *
 *	struct i915_user_extension ext3 {
 *		.next_extension = 0, // end
 *		.name = ...,
 *	};
 *	struct i915_user_extension ext2 {
 *		.next_extension = (uintptr_t)&ext3,
 *		.name = ...,
 *	};
 *	struct i915_user_extension ext1 {
 *		.next_extension = (uintptr_t)&ext2,
 *		.name = ...,
 *	};
 *
 * Typically the struct i915_user_extension would be embedded in some uAPI
 * struct, and in this case we would feed it the head of the chain(i.e ext1),
 * which would then apply all of the above extensions.
 *
 */
struct xe_user_extension {
	/**
	 * @next_extension:
	 *
	 * Pointer to the next struct i915_user_extension, or zero if the end.
	 */
	__u64 next_extension;
	/**
	 * @name: Name of the extension.
	 *
	 * Note that the name here is just some integer.
	 *
	 * Also note that the name space for this is not global for the whole
	 * driver, but rather its scope/meaning is limited to the specific piece
	 * of uAPI which has embedded the struct i915_user_extension.
	 */
	__u32 name;
	/**
	 * @flags: MBZ
	 *
	 * All undefined bits must be zero.
	 */
	__u32 pad;
};

/*
 * i915 specific ioctls.
 *
 * The device specific ioctl range is [DRM_COMMAND_BASE, DRM_COMMAND_END) ie
 * [0x40, 0xa0) (a0 is excluded). The numbers below are defined as offset
 * against DRM_COMMAND_BASE and should be between [0x0, 0x60).
 */
#define DRM_XE_DEVICE_QUERY	0x00
#define DRM_XE_GEM_CREATE	0x01
#define DRM_XE_GEM_MMAP_OFFSET	0x02
#define DRM_XE_VM_CREATE	0x03
#define DRM_XE_VM_DESTROY	0x04
#define DRM_XE_VM_BIND		0x05
#define DRM_XE_ENGINE_CREATE	0x06
#define DRM_XE_ENGINE_DESTROY	0x07
#define DRM_XE_EXEC		0x08
#define DRM_XE_MMIO		0x09

/* Must be kept compact -- no holes */
#define DRM_IOCTL_XE_DEVICE_QUERY	DRM_IOWR(DRM_COMMAND_BASE + DRM_XE_GEM_CREATE, struct drm_xe_device_query)
#define DRM_IOCTL_XE_GEM_CREATE		DRM_IOWR(DRM_COMMAND_BASE + DRM_XE_GEM_CREATE, struct drm_xe_gem_create)
#define DRM_IOCTL_XE_GEM_MMAP_OFFSET	DRM_IOWR(DRM_COMMAND_BASE + DRM_XE_GEM_MMAP_OFFSET, struct drm_xe_gem_mmap_offset)
#define DRM_IOCTL_XE_VM_CREATE		DRM_IOWR(DRM_COMMAND_BASE + DRM_XE_VM_CREATE, struct drm_xe_vm_create)
#define DRM_IOCTL_XE_VM_DESTROY		DRM_IOW( DRM_COMMAND_BASE + DRM_XE_VM_DESTROY, struct drm_xe_vm_destroy)
#define DRM_IOCTL_XE_VM_BIND		DRM_IOW( DRM_COMMAND_BASE + DRM_XE_VM_BIND, struct drm_xe_vm_bind)
#define DRM_IOCTL_XE_ENGINE_CREATE	DRM_IOWR(DRM_COMMAND_BASE + DRM_XE_ENGINE_CREATE, struct drm_xe_engine_create)
#define DRM_IOCTL_XE_ENGINE_DESTROY	DRM_IOW( DRM_COMMAND_BASE + DRM_XE_ENGINE_DESTROY, struct drm_xe_engine_destroy)
#define DRM_IOCTL_XE_EXEC		DRM_IOW( DRM_COMMAND_BASE + DRM_XE_EXEC, struct drm_xe_exec)
#define DRM_IOCTL_XE_MMIO		DRM_IOWR(DRM_COMMAND_BASE + DRM_XE_MMIO, struct drm_xe_mmio)

struct drm_xe_engine_class_instance {
	__u16 engine_class;

#define DRM_XE_ENGINE_CLASS_RENDER 0
#define DRM_XE_ENGINE_CLASS_COPY 1

	__u16 engine_instance;
};

struct drm_xe_device_query {
	/** @extensions: Pointer to the first extension struct, if any */
	__u64 extensions;

	/** @query: The type of data to query */
	__u32 query;

#define DRM_XE_DEVICE_QUERY_ENGINES 0

	/** @size: Size of the queried data */
	__u32 size;

	/** @data: Queried data is placed here */
	__u64 data;
};

struct drm_xe_gem_create {
	/** @extensions: Pointer to the first extension struct, if any */
	__u64 extensions;

	/**
	 * @size: Requested size for the object
	 *
	 * The (page-aligned) allocated size for the object will be returned.
	 */
	__u64 size;

	/** @flags: Flags */
	__u32 flags;

#define DRM_XE_GEM_CREATE_SYSTEM	0x1
#define DRM_XE_GEM_CREATE_VRAM		0x2

	/**
	 * @vm_id: Attached VM, if any
	 *
	 * If a VM is specified, this dma-buf must:
	 *
	 *  1. Only ever be bound to that VM.
	 *
	 *  2. Cannot be exported as a PRIME fd.
	 *
	 *  3. Cannot be used for implicit synchronization.
	 */
	__u32 vm_id;

	/**
	 * @handle: Returned handle for the object.
	 *
	 * Object handles are nonzero.
	 */
	__u32 handle;
};

struct drm_xe_gem_mmap_offset {
	/** @extensions: Pointer to the first extension struct, if any */
	__u64 extensions;

	/** @handle: Handle for the object being mapped. */
	__u32 handle;

	/** @flags: Must be zero */
	__u32 flags;

	/** @offset: The fake offset to use for subsequent mmap call */
	__u64 offset;
};

struct drm_xe_vm_create {
	/** @extensions: Pointer to the first extension struct, if any */
	__u64 extensions;

	/** @flags: MBZ */
	__u32 flags;

	/** @vm_id: Returned VM ID */
	__u32 vm_id;
};

struct drm_xe_vm_destroy {
	/** @vm_id: Returned VM ID */
	__u32 vm_id;

	/** @pad: MBZ */
	__u32 pad;
};

struct drm_xe_vm_bind {
	/** @extensions: Pointer to the first extension struct, if any */
	__u64 extensions;

	/** @vm_id: The ID of the VM to bind to */
	__u32 vm_id;

	/** @obj: GEM object to bind */
	__u32 obj;

	/** Offset into the object */
	__u64 offset;

	/** Number of bytes from the object to bind to addr */
	__u64 range;

	/** Address to bind to */
	__u64 addr;
};

struct drm_xe_engine_create {
	/** @extensions: Pointer to the first extension struct, if any */
	__u64 extensions;

	/** @instance: Physical HW engine */
	struct drm_xe_engine_class_instance instance;

	/** @vm_id: VM to use for this engine */
	__u32 vm_id;

	/** @flags: MBZ */
	__u32 flags;

	/** @engine_id: Returned engine ID */
	__u32 engine_id;
};

struct drm_xe_engine_destroy {
	/** @vm_id: Returned VM ID */
	__u32 engine_id;

	/** @pad: MBZ */
	__u32 pad;
};

struct drm_xe_sync {
	/** @extensions: Pointer to the first extension struct, if any */
	__u64 extensions;

	__u32 flags;

#define DRM_XE_SYNC_SYNCOBJ		0x0
#define DRM_XE_SYNC_TIMELINE_SYNCOBJ	0x1
#define DRM_XE_SYNC_DMA_BUF		0x2
#define DRM_XE_SYNC_SIGNAL		0x4

	__u32 handle;

	__u64 timeline_value;
};

struct drm_xe_exec {
	/** @extensions: Pointer to the first extension struct, if any */
	__u64 extensions;

	/** @vm_id: Returned VM ID */
	__u32 engine_id;

	__u32 num_syncs;

	__u64 syncs;

	__u64 address;
};

struct drm_xe_mmio {
	/** @extensions: Pointer to the first extension struct, if any */
	__u64 extensions;

	__u32 addr;

	__u32 flags;

#define DRM_XE_MMIO_8BIT	0x0
#define DRM_XE_MMIO_16BIT	0x1
#define DRM_XE_MMIO_32BIT	0x2
#define DRM_XE_MMIO_64BIT	0x3
#define DRM_XE_MMIO_BITS_MASK	0x3
#define DRM_XE_MMIO_READ	0x4
#define DRM_XE_MMIO_WRITE	0x8

	__u64 value;
};

#if defined(__cplusplus)
}
#endif

#endif /* _UAPI_XE_DRM_H_ */
