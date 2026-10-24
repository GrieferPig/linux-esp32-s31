/* SPDX-License-Identifier: GPL-2.0-only */
static struct sched_dl_entity *
s31_radio_fair_server_next(struct rq *rq, struct sched_dl_entity *dl_se)
{
	struct rb_node *node;

	lockdep_assert_rq_held(rq);
	if (dl_se != &rq->fair_server || !s31_radio_rt_candidate(rq))
		return dl_se;
	/* The fair server is the sole exception, regardless of EDF ordering.
	 * Keep its budget and timer state intact; normal selection resumes as
	 * soon as the actual RT candidate is no longer an admitted radio role. */
	node = rb_next(&dl_se->rb_node);
	return node ? __node_2_dle(node) : NULL;
}
