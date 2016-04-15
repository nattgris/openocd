/***************************************************************************
 *   Copyright (C) 2016 by Andreas Fritiofson                              *
 *   andreas.fritiofson@gmail.com                                          *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/time_support.h>
#include <jtag/jtag.h>
#include "target/target.h"
#include "target/target_type.h"
#include "rtos.h"
#include "helper/log.h"
#include "helper/types.h"
#include "rtos_standard_stackings.h"
#include "target/armv7m.h"
#include "target/cortex_m.h"

#define ST_DEAD     (1 << 0)    /* Task is waiting to be deleted */
#define ST_WAIT     (1 << 1)    /* Task is blocked: */
#define ST_SEM      (1 << 2)    /*  on semaphore */
#define ST_MTX      (1 << 3)    /*  on mutex */
#define ST_SIG      (1 << 4)    /*  on signal */
#define ST_DLY      (1 << 5)    /*  on timer */
#define ST_FLAG     (1 << 6)    /*  on flag */
#define ST_FLAG_ALL (1 << 7)    /*  on flag and flag mode is "ALL" */
#define ST_MBOX     (1 << 8)    /*  on mailbox */
#define ST_STP      (1 << 9)    /*  self stopped */
#define ST_SUSPEND  (1 << 10)   /* Task is suspended */
#define ST_TT       (1 << 11)   /* Time triggered task */
#define ST_TT_YIELD (1 << 12)   /* Time triggered task that yields */
#define ST_CREATE   (1 << 13)   /* Task was created by task_create() */

#define rtkernel_STRUCT(int_type, ptr_type, list_prev_offset)

struct rtkernel_params {
	const char *target_name;
	const struct rtos_register_stacking *stacking_info_cm3;
	const struct rtos_register_stacking *stacking_info_cm4f;
	const struct rtos_register_stacking *stacking_info_cm4f_fpu;
};

static const struct rtkernel_params rtkernel_params_list[] = {
	{
	"cortex_m",			/* target_name */
	&rtos_standard_Cortex_M3_stacking,	/* stacking_info */
	&rtos_standard_Cortex_M4F_stacking,
	&rtos_standard_Cortex_M4F_FPU_stacking,
	},
	{
	"hla_target",			/* target_name */
	&rtos_standard_Cortex_M3_stacking,	/* stacking_info */
	&rtos_standard_Cortex_M4F_stacking,
	&rtos_standard_Cortex_M4F_FPU_stacking,
	},
};

#define RTKERNEL_NUM_PARAMS ((int)(sizeof(rtkernel_params_list)/sizeof(struct rtkernel_params)))

static bool rtkernel_detect_rtos(struct target *target);
static int rtkernel_create(struct target *target);
static int rtkernel_update_threads(struct rtos *rtos);
static int rtkernel_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
		struct rtos_reg **reg_list, int *num_regs);
static int rtkernel_get_symbol_list_to_lookup(symbol_table_elem_t *symbol_list[]);

struct rtos_type rtkernel_rtos = {
	.name = "rtkernel",

	.detect_rtos = rtkernel_detect_rtos,
	.create = rtkernel_create,
	.update_threads = rtkernel_update_threads,
	.get_thread_reg_list = rtkernel_get_thread_reg_list,
	.get_symbol_list_to_lookup = rtkernel_get_symbol_list_to_lookup,
};

enum rtkernel_symbol_values {
	sym_os_state = 0,
	sym___off_os_state2chain = 1,
	sym___off_os_state2current = 2,
	sym___off_task2chain = 3,
	sym___off_task2magic = 4,
	sym___off_task2stack = 5,
	sym___off_task2state = 6,
	sym___off_task2name = 7,
	sym___val_task_magic = 8,
};

struct symbols {
	const char *name;
	bool optional;
};

static const struct symbols rtkernel_symbol_list[] = {
	{ "os_state", false },
	{ "__off_os_state2chain", false },
	{ "__off_os_state2current", false },
	{ "__off_task2chain", false },
	{ "__off_task2magic", false },
	{ "__off_task2stack", false },
	{ "__off_task2state", false },
	{ "__off_task2name", false },
	{ "__val_task_magic", false },
	{ NULL, false }
};

static int rtkernel_add_task(struct rtos *rtos, uint32_t task, uint32_t current_task)
{
	int retval;
	int new_thread_count = rtos->thread_count + 1;
	struct thread_detail *new_thread_details = realloc(rtos->thread_details,
			new_thread_count * sizeof(struct thread_detail));
	if (new_thread_details == NULL) {
		LOG_ERROR("Error growing memory to %d threads", new_thread_count);
		return ERROR_FAIL;
	}
	rtos->thread_details = new_thread_details;
	struct thread_detail *thread = &new_thread_details[rtos->thread_count];

	thread->threadid = task;
	thread->exists = true;

	/* Read the task name */
	uint32_t name;
	retval = target_read_u32(rtos->target, task + rtos->symbols[sym___off_task2name].address, &name);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not read task name pointer from target");
		return retval;
	}
	uint8_t tmp_str[32];
	retval = target_read_buffer(rtos->target, name, sizeof(tmp_str), tmp_str);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error reading task name from target");
		return retval;
	}
	tmp_str[sizeof(tmp_str)-1] = '\0';
	LOG_DEBUG("task name at 0x%" PRIx32 ", value \"%s\"", name, tmp_str);

	if (tmp_str[0] == '\0')
		strcpy((char *)tmp_str, "No Name");

	thread->thread_name_str = strdup((char *)tmp_str);

	/* Read the task state */
	uint16_t state;
	retval = target_read_u16(rtos->target, task + rtos->symbols[sym___off_task2state].address, &state);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not read task state from target");
		return retval;
	}

	LOG_DEBUG("task state 0x%" PRIx16, state);

	char state_str[64] = "";
	if (state & ST_TT)
		strcat(state_str, "TT|");
	if (task == current_task)
		strcat(state_str, "RUN");
	else {
		if (state == ST_TT)
			strcat(state_str, "READY");
		if (state & ST_DEAD)
			strcat(state_str, "DEAD");
		else if (state & ST_WAIT)
			strcat(state_str, "WAIT");
		else if (state & ST_SUSPEND)
			strcat(state_str, "SUSP");
		else
			strcat(state_str, "READY");
	}
	if (state & ST_SEM)
		strcat(state_str, "|SEM");
	if (state & ST_MTX)
		strcat(state_str, "|MTX");
	if (state & ST_SIG)
		strcat(state_str, "|SIG");
	if (state & ST_DLY)
		strcat(state_str, "|DLY");
	if ((state & ST_FLAG) || (state & ST_FLAG_ALL))
		strcat(state_str, "|FLAG");
	if (state & ST_FLAG_ALL)
		strcat(state_str, "_ALL");
	if (state & ST_MBOX)
		strcat(state_str, "|MBOX");
	if (state & ST_STP)
		strcat(state_str, "|STP");

	thread->extra_info_str = strdup(state_str);

	rtos->thread_count = new_thread_count;
	if (task == current_task)
		rtos->current_thread = task;
	return ERROR_OK;
}

static int rtkernel_verify_task(struct rtos *rtos, uint32_t task)
{
	int retval;
	uint32_t magic;
	retval = target_read_u32(rtos->target, task + rtos->symbols[sym___off_task2magic].address, &magic);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not read task magic from target");
		return retval;
	}
	if (magic != rtos->symbols[sym___val_task_magic].address) {
		LOG_ERROR("Invalid task found (magic=0x%"PRIx32")", magic);
		return ERROR_FAIL;
	}
	return retval;
}

static int rtkernel_update_threads(struct rtos *rtos)
{
	int retval;

	/* wipe out previous thread details if any */
	/* do this first because rtos layer does not check our retval */
	rtos_free_threadlist(rtos);
	rtos->current_thread = 0;

	if (rtos->symbols == NULL) {
		LOG_ERROR("No symbols for rt-kernel");
		return -3;
	}

	/* read the current task */
	uint32_t current_task;
	retval = target_read_u32(rtos->target,
			rtos->symbols[sym_os_state].address + rtos->symbols[sym___off_os_state2current].address,
			&current_task);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error reading current task");
		return retval;
	}
	LOG_DEBUG("current task is 0x%"PRIx32, current_task);

	retval = rtkernel_verify_task(rtos, current_task);
	if (retval != ERROR_OK) {
		LOG_ERROR("Current task is invalid");
		return retval;
	}

	/* loop through kernel task list */
	uint32_t chain = rtos->symbols[sym_os_state].address + rtos->symbols[sym___off_os_state2chain].address;
	LOG_DEBUG("chain start at 0x%"PRIx32, chain);

	uint32_t next = chain;
	for (;;) {
		retval = target_read_u32(rtos->target, next, &next);
		if (retval != ERROR_OK) {
			LOG_ERROR("Could not read rt-kernel data structure from target");
			return retval;
		}
		LOG_DEBUG("next entry at 0x%" PRIx32, next);
		if (next == chain) {
			LOG_DEBUG("end of chain detected");
			break;
		}
		uint32_t task = next - rtos->symbols[sym___off_task2chain].address;
		LOG_DEBUG("found task at 0x%" PRIx32, task);

		retval = rtkernel_verify_task(rtos, task);
		if (retval != ERROR_OK) {
			LOG_ERROR("Invalid task found");
			return retval;
		}

		retval = rtkernel_add_task(rtos, task, current_task);
		if (retval != ERROR_OK) {
			LOG_ERROR("Could not add task to rtos system");
			return retval;
		}
	}
	return 0;
}

static int rtkernel_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
		struct rtos_reg **reg_list, int *num_regs)
{
	int retval;
	const struct rtkernel_params *param;
	uint32_t stack_ptr = 0;

	if (rtos == NULL)
		return -1;

	if (thread_id == 0)
		return -2;

	if (rtos->rtos_specific_params == NULL)
		return -1;

	param = (const struct rtkernel_params *) rtos->rtos_specific_params;

	/* Read the stack pointer */
	retval = target_read_u32(rtos->target, thread_id + rtos->symbols[sym___off_task2stack].address, &stack_ptr);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error reading stack pointer from rtkernel thread");
		return retval;
	}
	LOG_DEBUG("stack pointer at 0x%" PRIx64 ", value 0x%" PRIx32,
			thread_id + rtos->symbols[sym___off_task2stack].address,
			stack_ptr);

	/* Adjust stack pointer to ignore non-standard BASEPRI register stacking */
	stack_ptr += 4;

	/* Check for armv7m with *enabled* FPU, i.e. a Cortex M4F */
	int cm4_fpu_enabled = 0;
	struct armv7m_common *armv7m_target = target_to_armv7m(rtos->target);
	if (is_armv7m(armv7m_target)) {
		if (armv7m_target->fp_feature == FPv4_SP) {
			/* Found ARM v7m target which includes a FPU */
			uint32_t cpacr;

			retval = target_read_u32(rtos->target, FPU_CPACR, &cpacr);
			if (retval != ERROR_OK) {
				LOG_ERROR("Could not read CPACR register to check FPU state");
				return -1;
			}

			/* Check if CP10 and CP11 are set to full access. */
			if (cpacr & 0x00F00000) {
				/* Found target with enabled FPU */
				cm4_fpu_enabled = 1;
			}
		}
	}

	if (cm4_fpu_enabled == 1) {
		/* Read the LR to decide between stacking with or without FPU */
		uint32_t LR_svc;
		retval = target_read_u32(rtos->target, stack_ptr + 0x20, &LR_svc);
		if (retval != ERROR_OK) {
			LOG_OUTPUT("Error reading stack frame from rtkernel thread\r\n");
			return retval;
		}
		if ((LR_svc & 0x10) == 0) {
			LOG_DEBUG("cm4f_fpu stacking");
			return rtos_generic_stack_read(rtos->target, param->stacking_info_cm4f_fpu, stack_ptr, reg_list, num_regs);
		} else {
			LOG_DEBUG("cm4f stacking");
			return rtos_generic_stack_read(rtos->target, param->stacking_info_cm4f, stack_ptr, reg_list, num_regs);
		}
	} else {
		LOG_DEBUG("cm3 stacking");
		return rtos_generic_stack_read(rtos->target, param->stacking_info_cm3, stack_ptr, reg_list, num_regs);
	}
}

static int rtkernel_get_symbol_list_to_lookup(symbol_table_elem_t *symbol_list[])
{
	unsigned int i;
	*symbol_list = calloc(
			ARRAY_SIZE(rtkernel_symbol_list), sizeof(symbol_table_elem_t));

	for (i = 0; i < ARRAY_SIZE(rtkernel_symbol_list); i++) {
		(*symbol_list)[i].symbol_name = rtkernel_symbol_list[i].name;
		(*symbol_list)[i].optional = rtkernel_symbol_list[i].optional;
	}

	return 0;
}

static bool rtkernel_detect_rtos(struct target *target)
{
	return (target->rtos->symbols != NULL) &&
			(target->rtos->symbols[sym___off_os_state2chain].address != 0);
}

static int rtkernel_create(struct target *target)
{
	for (int i = 0; i < RTKERNEL_NUM_PARAMS; i++) {
		if (strcmp(rtkernel_params_list[i].target_name, target->type->name) == 0) {
			target->rtos->rtos_specific_params = (void *) &rtkernel_params_list[i];
			return 0;
		}
	}

	LOG_ERROR("Could not find target in rt-kernel compatibility list");
	return -1;
}
