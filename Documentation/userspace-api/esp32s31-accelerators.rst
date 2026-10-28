ESP32-S31 memory accelerators
============================

JPEG and PPA use V4L2 memory-to-memory video nodes named ``s31-jpeg`` and
``s31-ppa``. Discover nodes through ``VIDIOC_QUERYCAP``; video numbers depend
on probe order. Queue one OUTPUT and one CAPTURE buffer using the standard
REQBUFS/MMAP/QBUF/STREAMON/DQBUF flow. No camera, display or userspace daemon
is required. MMAP buffers are allocated on demand, with at most two buffers
per queue. REQBUFS(0) drops queue ownership. Closing the last video-file
reference does the same, but video-node mappings keep that file alive after
its descriptor closes. Use STREAMOFF to explicitly stop DMA while mappings
remain. Mappings and exported DMA-BUF references retain backing memory until
the last reference is released; independent DMA-BUF mappings do not retain
the original video file. Queue setup allocates no global frame pool.
Cached MMAP buffers use vb2 DMA synchronization at queue/dequeue;
For MMAP, REQBUFS/CREATE_BUFS return V4L2_MEMORY_FLAG_NON_COHERENT.
Applications need
no manual cache operations for MMAP buffers. Standard EXPBUF and DMABUF
queues allow an application to reuse the same allocation between engines.
Bracket CPU access through an exported DMA-BUF with DMA_BUF_IOCTL_SYNC;
wait for DQBUF before accessing a buffer or passing it to another engine.
The driver uses explicit queue completion, not implicit DMA-BUF fences.

Keep the file, buffer allocations and streaming queues open across frames.
For a synchronous producer, open without O_NONBLOCK and wait in DQBUF after
queueing both buffers; this avoids a failed nonblocking dequeue and a separate
poll round trip. Event-driven applications can continue to use nonblocking
dequeue and poll. Neither path requires a userspace worker or daemon.

MMAP queues advertise V4L2_BUF_CAP_SUPPORTS_MMAP_CACHE_HINTS. After an ordinary
DMA handoff, a buffer which the CPU has not written may be queued with
V4L2_BUF_FLAG_NO_CACHE_CLEAN to avoid a redundant writeback. This also applies
to a reused CAPTURE buffer which the CPU has only read. Do not set this hint
after writing any part of the allocation, including pixels outside a PPA
composition rectangle. Keep capture invalidation enabled when the CPU reads
the result. V4L2_BUF_FLAG_NO_CACHE_INVALIDATE is for buffers passed between
engines without CPU access; explicit DMA-BUF synchronization is still
required before later CPU access. Cache hints do not replace queue completion
or change buffer lifetime.
Source and destination image canvases must not overlap in physical memory,
including when separate queue slots import the same DMA-BUF. Such a job
returns its buffers with ERROR before either engine touches the payload.

Dimensions are multiples of 16, up to 1024x768. Applications must use the
returned format dimensions, bytesperline and sizeimage. JPEG compressed
buffers are bounded to 1 MiB. Insufficient memory returns an allocation
error. The board configuration provides a shared 4 MiB CMA area, whose
unused pages remain available for movable allocations. It is not a dedicated
frame pool. This accommodates three VGA RGB24 canvases for composition.
Page blocks are 2 MiB and allocation alignment is capped at 16 KiB to avoid
wasting the small area on alignment padding.

Buffers must be physically contiguous, so fragmentation can cause allocation
failure even when MemAvailable exceeds the requested size. Negotiate the
compressed capacity for the workload and allocate reusable buffers before
memory becomes fragmented; an allocation error does not stop either engine.

JPEG OUTPUT/CAPTURE accept JPEG, GREY, RGB24 and BGR24. Exactly one queue must
use JPEG. Encoder dimensions must match on both queues. The encoder quality
control is V4L2_CID_JPEG_COMPRESSION_QUALITY, 1..100. Color encoding uses 4:2:0,
and GREY uses a one-component baseline scan. Decoding accepts one baseline
8-bit scan, gray/4:4:4/4:2:2/4:2:0, standard Y/chroma Huffman lanes and matching
negotiated dimensions. Adobe RGB/YCCK-marked input, progressive/multi-scan, other table mappings and
unsupported sizes are rejected before DMA. The hardware's packed YUV420 is
not advertised as Linux planar YU12. Color decode is converted by DMA2D
using full-range JPEG YCbCr coefficients.

PPA accepts RGB24, BGR24, RGB565, ABGR32, RGBA32, GREY and UYVY, using
standard FOURCC memory layouts. Rotation (counterclockwise), horizontal/vertical mirroring
use V4L2_CID_ROTATE/HFLIP/VFLIP. VIDIOC_G_SELECTION/S_SELECTION configure
an OUTPUT crop and a CAPTURE
composition rectangle before buffer allocation; S_FMT resets that queue's
rectangle to the full image. DEFAULT/BOUNDS report the full image. Selection
width and height are multiples of 16. UYVY requires an even horizontal crop
and composition offset. The receive port writes halfwords, so GREY/RGB24/BGR24
also require an even horizontal composition offset. Normal selection requests
are adjusted; GE/LE constrain containment and incompatible exact requests
return ERANGE. Pixels outside the composition rectangle are retained. The
standard V4L2_CID_ALPHA_COMPONENT control supplies alpha for an
ABGR32/RGBA32 output
when the input has no alpha channel; an input alpha channel is preserved.

Scaling is derived from the crop and composition rectangle
sizes; hardware factors have 1/16 precision, so a pair of dimensions
which cannot be represented returns an error.

The standard COLORFX control accepts NONE (normal SRM) and SET_RGB. SET_RGB
uses the PPA fill engine, replacing RGB with COLORFX_RGB's 0xRRGGBB constant
inside the composition rectangle. The source buffer still participates in
the M2M queue protocol, but receive-only fill does not DMA-read its pixels;
a reusable 16x16 GREY source is sufficient. The source is never modified.
RGB565 is quantized to its native component widths. GREY and UYVY constants
are converted once using full-range BT.601 coefficients. Outputs with alpha
use ALPHA_COMPONENT when the source has no alpha. An alpha-bearing source
to alpha-bearing destination is rejected in SET_RGB mode, because fill cannot
preserve a varying source alpha channel. Other unsupported color-effect menu
entries are disabled. Two-image blending uses the separate media pipeline
described below, without a custom blending ABI or physical-address interface.
RGB channel order is handled in SRM without an additional DMA conversion
stage. UYVY input to RGB24/RGBA32 output is rejected; select native
BGR24/ABGR32 instead.
For a 32-bit alpha-bearing input, the hardware input swap reverses all four
bytes. Conversions requiring a red/blue swap would also move or discard alpha;
these pairs are rejected. ABGR32 can target native BGR24/ABGR32, and RGBA32 can
target RGB24/RGBA32, without that swap. Three-byte inputs can generate either
alpha layout using the standard constant-alpha control.

RGB565 expansion fills the low RGB component bits with zero. UYVY passes
through the engine's color conversion, including when both queues use UYVY;
conversion rounding can change samples. It is not a lossless byte-copy path.

Solid-color generation also has a capture-only node named ``s31-ppa-fill``.
It uses the same PPA engine and DMA ownership rules, with one CAPTURE queue
and no OUTPUT buffer. Discover it through QUERYCAP rather than a fixed video
number. Its standard TEST_PATTERN menu has one item, Solid Color. Set the
8-bit TEST_PATTERN_RED, TEST_PATTERN_GREENR (the RGB green component),
TEST_PATTERN_BLUE and ALPHA_COMPONENT controls. Formats and CAPTURE composition
selection follow the PPA rules above; changing format resets the composition
rectangle. Colors and alpha are latched by QBUF, so subsequent control
changes do not alter queued frames.
One file owns the capture queue; other files cannot replace its buffers.

This producer is demand-driven: it generates a frame for each queued capture
buffer, without a camera, display clock or frame-rate timer. It supports
MMAP, DMA-BUF import/export and blocking or nonblocking DQBUF, with one or two
buffers. Timestamps are monotonic completion times and sequence numbers restart
at STREAMON. read()/write() file I/O is not advertised. The generator and M2M
PPA operations share the same hardware lock and runtime/system power management.

Two-image composition is exposed by the ``S31 PPA compositor`` media device.
Its immutable graph has ``s31-ppa-background`` and ``s31-ppa-foreground``
VIDEO_OUTPUT nodes connected to sink pads 0 and 1 of ``s31-ppa-blend``, whose
source pad 2 connects to the ``s31-ppa-composed`` VIDEO_CAPTURE node. Discover
entities and device numbers using MEDIA_IOC_G_TOPOLOGY and QUERYCAP. The
read-only subdevice exposes G_FMT ACTIVE for each pad, describing the selected
block's dimensions and bus format; memory layout and selection are configured
through the corresponding video nodes.

Set all three memory formats and rectangles before allocating any buffers.
Use OUTPUT CROP independently on both inputs, and CAPTURE COMPOSE on the
output. The three selected rectangles must have identical sizes: blending
does not scale them. Input and output canvases and offsets may differ.
Geometry is immutable while any of the pipeline's queues owns buffers.
Each node has one queue owner and supports one or two MMAP or imported
DMA-BUF buffers; MMAP buffers may also be exported. Applications coordinate
the three queue owners, queue full-sized input payloads, then STREAMON all
three nodes in any order. A job consumes one buffer from each queue in FIFO
order. Its three completion timestamps and sequence numbers match. Sequence
numbers restart after all three nodes have stopped streaming.

The input ALPHA_COMPONENT control is snapshotted by QBUF. Without a pixel
alpha channel it supplies the fixed opacity. With a pixel alpha channel,
255 preserves it; values 0..254 apply the hardware's alpha/256 multiplier.
Pixels use straight, unassociated alpha. PREMUL_ALPHA formats are rejected.
The hardware implements foreground source-over composition, normalizing the
result by its resulting alpha. RGB components are unspecified when the
resulting alpha is zero. Foreground accepts RGB24, BGR24, RGB565, ABGR32
and RGBA32. Background and capture additionally accept GREY and UYVY.
The existing PPA channel-order restrictions apply: an alpha-bearing source
cannot change between red-first and blue-first layouts, and UYVY background
cannot feed a red-first output. STREAMON rejects incompatible rectangles
or channel orders before DMA starts. Indexed palettes, alpha-only formats
and color keying are not exposed by this interface.

Stopping or closing any queue waits for the bounded active DMA job and
returns all pending triples with ERROR, including buffers on peer queues.
Peer queues remain streaming; the application must dequeue their errors
and requeue before restarting the stopped member. The standard video-file
VMA lifetime and DMA-BUF synchronization rules above apply to all three
queues. A missing member or an empty queue never causes polling: the worker
is scheduled only when all three queues are streaming and have a buffer.
Two input DMA channels and one output channel share the existing DMA2D
lease, interrupt completion, bounded recovery and PM with other media and
DMAengine users. No source image is copied into a combined software buffer.

DMA2D also registers a native DMAengine provider for kernel consumers,
including memory copies and single-chunk interleaved rectangular copies.
Use streaming DMA mappings and the corresponding CPU/device ownership
transfers for payload buffers. The current RV32 memory mapping does not
provide an uncached alias for ``dma_alloc_coherent()``; coherent allocations
are not supported for these payloads. The V4L2 interfaces above use the
noncoherent allocator and perform the required synchronization internally.
The same provider serializes codec and PPA transport, owns a 256-byte SRAM
descriptor reservation, sleeps for IRQ completion, and uses bounded recovery.
This is distinct from the already implemented AXI/AHB GDMA engines.

Custom fixed-size interfaces
---------------------------

``linux/esp32s31-accelerators.h`` defines versioned ioctls with aligned 64-bit
user pointers and reserved fields which must be zero. No physical addresses,
register mappings, executable kernel code or arbitrary DMA descriptors are
accepted. Devices default to mode 0600. These are S31-specific APIs, not
upstream framework ABIs.

``/dev/s31-cordic`` accepts S31_CORDIC_RUN batches of up to 4096 samples.
Function values 0..9 select cosine, sine, phase, modulus, atan, cosh, sinh,
atanh, natural log and square root. Formats 0/1 select Q15/Q31. Circular
angles use fixed-point fractions of pi; function-specific scale follows
the S31 hardware convention. Phase/modulus use both arguments. Iterations
range from 1 to 6; higher values are unsupported and rejected. The
function/scale combination is validated. Results
are signed integers in the selected format; Q15 uses signed low halves.
``completed`` identifies the returned prefix if a batch is interrupted or
hardware times out. Register-mode polling is bounded per sample, following
the SDK's short-operation path, with rescheduling between groups of samples.
Clocks are retained for a short autosuspend window to amortize setup.

``/dev/s31-etm`` accepts ROUTE_SET/GET/DEL. A new route requests AUTO_CHANNEL;
SET returns its file-owned channel. GET reports sticky event/task status,
not a counter. Closing the file disables and releases every owned channel.
Routes survive retained system sleep and are restored after clocks/reset.
Opening and modifying routes requires CAP_SYS_RAWIO because a task can
change another peripheral's state. The caller must arrange endpoint
configuration and ownership through the corresponding peripheral driver;
the router does not claim pins, configure DMA or keep endpoint clocks on.
No routes are configured automatically. Endpoint IDs follow the S31 SDK
soc_etm_source hardware table.

``/dev/s31-bitscrambler`` accepts S31_BITSCRAMBLER_RUN with an SDK bsasm-v1
program, input and output pointers. Instructions/LUT and buffer sizes are
validated, arbitrary program loops have a bounded timeout, and all DMA is
stopped/joined before buffers are returned. Both input and capacity must be
multiples of four, up to 256 KiB. ``written`` is received length, not offered
capacity. Memory loopback borrows the unused LCD GDMA trigger, never the
AES/SHA/radio routes. A future LCD DMA driver must coordinate that route
before enabling both consumers. Programs are reloaded and the entire LUT
is cleared for each operation. No program is retained across suspend.

For disjoint input/output streams of at least one page, with both addresses
and both lengths aligned to 64 bytes, RUN pins the userspace pages and uses
DMA scatter/gather directly. No payload bounce buffer or CPU copy is used.
Pages are unmapped, marked dirty where needed and unpinned after DMA and its
callbacks have stopped. The application must not access either stream until
RUN returns. Unaligned or overlapping streams use owned buffers to preserve
cache-line ownership and input snapshot semantics. This includes physical
overlap through different virtual mappings of the same pages. Pinning/mapping failures
return an error. Memory remains bounded by the 256 KiB per-stream limit.

Native applications use the UAPI directly and may batch CORDIC calls. These paths
do not require patches to libc or multimedia libraries. Automatic libm,
memcpy or JPEG-library interception is not provided.
