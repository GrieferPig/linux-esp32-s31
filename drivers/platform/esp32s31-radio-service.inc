/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * The Linux kthread worker owns dispatch and one persistent init_mm borrow.
 * There is no private scheduler or delayed-work timer. IRQ/service deadlines
 * continue to use the existing hrtimer and source-specific executors.
 */
static int s31_radio_service_thread(void *opaque)
{
	int result, final;

	if (raw_smp_processor_id() != 0 ||
	    !cpumask_equal(current->cpus_ptr, cpumask_of(0)))
		panic("esp32s31-radio: private service escaped core0");
	esp32s31_kthread_use_init_mm();

	result = kthread_worker_fn(opaque);
	/*
	 * Retire on the owner's Linux stack before returning init_mm. The
	 * worker is already stopped and unpublished, while the payload and
	 * SRAM mappings still exist. This consumes its exact exit guard once.
	 */
	final = s31_foreign_owner_finish_current();
	if (final)
		panic("esp32s31-radio: owned service finalization failed (%d)", final);
	final = s31_linux_radio_service_retire_current();
	if (final)
		panic("esp32s31-radio: service architecture retirement failed (%d)", final);

	esp32s31_kthread_unuse_init_mm();
	if (current->mm)
		panic("esp32s31-radio: private service leaked init_mm borrow");
	return result;
}

static struct kthread_worker *s31_radio_service_create(int cpu)
{
	struct kthread_worker *worker;
	struct task_struct *thread;

	if (cpu != -1 && cpu != 0)
		return ERR_PTR(-EINVAL);
	if (!cpu_online(0))
		return ERR_PTR(-ENODEV);
	worker = kzalloc(sizeof(*worker), GFP_KERNEL);
	if (!worker)
		return ERR_PTR(-ENOMEM);
	kthread_init_worker(worker);
	thread = kthread_create(s31_radio_service_thread, worker, "s31-radio/0");
	if (IS_ERR(thread)) {
		kfree(worker);
		return ERR_CAST(thread);
	}
	worker->task = thread;
	kthread_bind(thread, 0);
	/* Setup admits the dormant CPU0 worker at the fixed radio RR priority.
	 * Its normal nice level is irrelevant to this mandatory RT policy. */
	if (s31_linux_radio_service_setup(thread)) {
		kthread_stop(thread);
		kfree(worker);
		return ERR_PTR(-EINVAL);
	}
	wake_up_process(thread);
	return worker;
}

static void s31_radio_service_discard(void)
{
	struct kthread_worker *worker;
	unsigned long flags;

	/*
	 * Startup failure has not admitted work (state remains OFF). Still
	 * unpublish under the same lock used by every atomic producer.
	 */
	spin_lock_irqsave(&s31_radio_q_lock, flags);
	worker = s31_radio_worker;
	s31_radio_worker = NULL;
	spin_unlock_irqrestore(&s31_radio_q_lock, flags);
	if (worker)
		kthread_destroy_worker(worker);
}
