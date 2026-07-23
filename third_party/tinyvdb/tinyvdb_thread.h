#pragma once

#ifdef TINYVDB_ENABLE_THREAD
#include <threads.h>

typedef struct
{
  void (*func)(int, void*);
  void* data;
  int start;
  int end;
} tvdb_parallel_for_task_t;

static int tvdb__thread_worker(void* arg)
{
  tvdb_parallel_for_task_t* task = (tvdb_parallel_for_task_t*)arg;
  for (int i = task->start; i < task->end; ++i)
  {
    task->func(i, task->data);
  }
  return 0;
}

static void tvdb_parallel_for(int start, int end, void (*func)(int, void*), void* data)
{
  int num_threads = 4;  // Simplified: fixed count
  thrd_t threads[4];
  tvdb_parallel_for_task_t tasks[4];
  int range = (end - start) / num_threads;
  for (int i = 0; i < num_threads; i++)
  {
    tasks[i] = (tvdb_parallel_for_task_t){
      func, data, start + i * range, (i == num_threads - 1) ? end : start + (i + 1) * range
    };
    thrd_create(&threads[i], tvdb__thread_worker, &tasks[i]);
  }
  for (int i = 0; i < num_threads; i++)
  {
    thrd_join(threads[i], NULL);
  }
}
#else
static void tvdb_parallel_for(int start, int end, void (*func)(int, void*), void* data)
{
  for (int i = start; i < end; ++i)
    func(i, data);
}
#endif
