#include <linux/swap.h>
#include <linux/module.h>
#include <trace/hooks/binder.h>
#include <uapi/linux/android/binder.h>
#include <uapi/linux/sched/types.h>
#include <linux/sched/prio.h>
#include <../../android/binder_internal.h>
#include <linux/sched/cputime.h>
#include <../../../kernel/sched/sched.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/ftrace.h>

static int prio_debug;
module_param(prio_debug, uint, 0644);

enum binder_transaction_tag {
    BINDER_PIPELINE_HOME = 0,
    BINDER_PIPELINE_SYSTEMUI,
    BINDER_PIPELINE_SF,
    BINDER_PIPELINE_OTHER,   // 其他来源的 Binder 事务
};
#define MASK_BINDER_PIPELINE_HOME               (1 << BINDER_PIPELINE_HOME)
#define MASK_BINDER_PIPELINE_SYSTEMUI           (1 << BINDER_PIPELINE_SYSTEMUI)
#define MASK_BINDER_PIPELINE_SF                 (1 << BINDER_PIPELINE_SF)

static const char * const task_name[] = {
        "com.miui.home",
        "ndroid.systemui",
        "surfaceflinger",
        "cameraserver",
        "rsonalassistant",
        ".globallauncher",
        "cameraserver",
        "ndroid.settings",
        ".smile.gifmaker",
        ".ugc.aweme.lite",
        "tv.danmaku.bili",
        "com.UCMobile",
        "om.tencent.news",
        "v.douyu.android",
        "v.imgo.activity",
        "tencent.qqmusic",
        "kuaishou.nebula",
        ".tencent.qqlive",
        "com.qiyi.video",
        "com.youku.phone",
        "m.kugou.android",
        "com.baidu.tieba",
        "com.taobao.taobao",
        "com.sina.weibo",
        "com.tencent.mm",
        "id.AlipayGphone",
        "droid.ugc.aweme",
        "id.article.news",
        "ngdong.app.mall",
};
static const char *RenderThread = "RenderThread";
static const char *passBlur = "passBlur";
static const char *cameraserver_C3Dev = "C3Dev-";
static const char *cameraserver_ReqQ = "-ReqQ";
static const char *wmshell_main = "wmshell.main";
static const char *com_android_systemui = "ndroid.systemui";
static const char *wmshell_splashworker = "ll.splashscreen";
static const char *allocateBuffers = "allocateBuffers";

static int to_userspace_prio(int policy, int kernel_priority) {
        if (fair_policy(policy))
                return PRIO_TO_NICE(kernel_priority);
        else
                return MAX_RT_PRIO - 1 - kernel_priority;
}

static bool set_binder_rt_task(struct binder_transaction *t) {
        int i;
        if (!t || !t->from || !t->from->task || !t->to_proc || !t->to_proc->tsk) {
                return false;
        }

        if (t->flags & TF_ONE_WAY) {
                return false;
        }

        if (!rt_policy(t->from->task->policy)) {
                return false;
        }

        if ((strncmp(t->from->task->group_leader->comm, task_name[0], strlen(task_name[0])) == 0)
                && (strncmp(t->from->task->comm, RenderThread, strlen(RenderThread)) == 0)
                && (strncmp(t->to_proc->tsk->comm, task_name[2], strlen(task_name[2])) == 0)) {
                return true;
        }

        if ((strncmp(t->from->task->group_leader->comm, task_name[2], strlen(task_name[2])) == 0)
                && (strncmp(t->from->task->comm, passBlur, strlen(passBlur)) == 0)) {
                return true;
        }

        if ((strncmp(t->from->task->group_leader->comm, task_name[3], strlen(task_name[3])) == 0)
                && (strncmp(t->from->task->comm, cameraserver_C3Dev, strlen(cameraserver_C3Dev)) == 0)
                && (strstr(t->from->task->comm, cameraserver_ReqQ) != NULL)) {
                return true;
        }

        if (t->from->task->pid == t->from->task->tgid) {
                for(i = 0; i < sizeof(task_name)/sizeof(task_name[0]); i++) {
                        if (strncmp(t->from->task->comm, task_name[i], strlen(task_name[i])) == 0) {
                                return true;
                        }
                }
                return false;
        }
        return false;
}

static bool is_splashworker_task(struct binder_transaction *t) {
	if (!t || !t->from || !t->from->task || !t->to_proc || !t->to_proc->tsk) {
		return false;
	}

	if (t->flags & TF_ONE_WAY) {
		return false;
	}

	if (!rt_policy(t->from->task->policy)) {
	      return false;
	}

        if ((strncmp(t->from->task->group_leader->comm, com_android_systemui,
                     strlen(com_android_systemui)) == 0)
                && ((strncmp(t->from->task->comm, wmshell_main,
                             strlen(wmshell_main)) == 0)
                || (strncmp(t->from->task->comm, wmshell_splashworker,
                            strlen(wmshell_splashworker)) == 0)
                || (strncmp(t->from->task->comm, allocateBuffers,
                            strlen(allocateBuffers)) == 0))){
                        return true;
        }

	return false;
}

static bool binder_pipeline_task(struct binder_transaction *t)
{
        return t->android_vendor_data1 & (MASK_BINDER_PIPELINE_HOME | MASK_BINDER_PIPELINE_SYSTEMUI
                                        | MASK_BINDER_PIPELINE_SF);
}
static void binder_pipeline_transfer(struct binder_transaction *t)
{
        bool oneway = !!(t->flags & TF_ONE_WAY);
        struct binder_transaction *from = t->from_parent;
        if (oneway || !from) return;
        t->android_vendor_data1 = t->from_parent->android_vendor_data1;
}

static void extend_binder_proc_transaction_handler(void *d, struct task_struct *caller_task,
					    struct task_struct *binder_proc_task,
					    struct task_struct *binder_th_task, int node_debug_id,
					    struct binder_transaction *t, bool pending_async)
{
        binder_pipeline_transfer(t);
}

static void extend_surfacefinger_binder_set_priority_handler(void *data,
                struct binder_transaction *t, struct task_struct *task) {
        struct sched_param params;
        struct binder_priority desired;
        unsigned int policy;
        struct binder_node *target_node = t->buffer->target_node;

        desired.prio = target_node->min_priority;
        desired.sched_policy = target_node->sched_policy;
        policy = desired.sched_policy;
        if (set_binder_rt_task(t) || binder_pipeline_task(t) || is_splashworker_task(t)) {
                desired.sched_policy = SCHED_FIFO;
                desired.prio = 98;
                policy = desired.sched_policy;
        }
        if (rt_policy(policy) && task->policy != policy) {
                params.sched_priority = to_userspace_prio(policy, desired.prio);
                sched_setscheduler_nocheck(task, policy | SCHED_RESET_ON_FORK, &params);
        }
}

static void extend_surfacefinger_binder_trans_handler(void *data, struct binder_proc *target_proc,
				struct binder_proc *proc, struct binder_thread *thread, struct binder_transaction_data *tr) {
        if (target_proc && target_proc->tsk && strncmp(target_proc->tsk->comm, "surfaceflinger",
                strlen("surfaceflinger")) == 0) {
                if (thread && proc && tr && thread->transaction_stack
                        && (!(thread->transaction_stack->flags & TF_ONE_WAY))) {
                        target_proc->default_priority.sched_policy = SCHED_FIFO;
                        target_proc->default_priority.prio = 98;
                }
        }
}

static void extend_trace_android_vh_binder_transaction_init(void *nouse, struct binder_transaction *t)
{
    struct task_struct *current_task = current;
    const char *comm = current_task->comm;
    for (int i = 0; i < ARRAY_SIZE(task_name); i++) {
        if (strstr(comm, task_name[i]) != NULL) {
            t->android_vendor_data1 |= 1 << i;
            if (unlikely(prio_debug))
                pr_info("Binder transaction tagged: comm=%s, tag=%llu\n", comm, t->android_vendor_data1);
            return;
        }
    }
}

int __init binder_prio_init(void)
{
    pr_info("binder_prio: module init!");
    register_trace_android_vh_binder_set_priority(extend_surfacefinger_binder_set_priority_handler, NULL);
    register_trace_android_vh_binder_trans(extend_surfacefinger_binder_trans_handler, NULL);
    register_trace_android_vh_binder_transaction_init(extend_trace_android_vh_binder_transaction_init, NULL);
    register_trace_android_vh_binder_proc_transaction(extend_binder_proc_transaction_handler, NULL);
    return 0;
}

void __exit binder_prio_exit(void)
{
    unregister_trace_android_vh_binder_set_priority(extend_surfacefinger_binder_set_priority_handler, NULL);
    unregister_trace_android_vh_binder_trans(extend_surfacefinger_binder_trans_handler, NULL);
    unregister_trace_android_vh_binder_proc_transaction(extend_binder_proc_transaction_handler, NULL);
    unregister_trace_android_vh_binder_transaction_init(extend_trace_android_vh_binder_transaction_init, NULL);
    pr_info("binder_prio: module exit!");
}

module_init(binder_prio_init);
module_exit(binder_prio_exit);
MODULE_LICENSE("GPL");

