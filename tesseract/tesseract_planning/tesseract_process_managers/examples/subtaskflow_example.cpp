#include <taskflow/taskflow.hpp>
#include <tesseract_common/utils.h>

struct TaskInput
{
  int* cnt;
};

/**
 * @brief This example is to explore the dynamic tasking functionality of taskflow.
 * @details The first tasks will change a variable which is then used by a subsequent task to dynamically create n
 * number of tasks.
 * @return
 */
int main(int /*argc*/, char** /*argv*/)
{
  tf::Executor executor;
  tf::Taskflow taskflow;

  int num_subtaskflows = 1;
  TaskInput input;
  input.cnt = &num_subtaskflows;

  tf::Task t = taskflow.placeholder();
  std::cout << t.hash_value() << std::endl;
  t.work([=]() { *(input.cnt) = 10; });
  std::cout << t.hash_value() << std::endl;

  // The default process input has a default count of 1 and when this task is ran it will change the count to 10,
  // then in B taskflow at runtime will read this variable and dynamically generate the number of task set in the
  // process input which should be 10.
  tf::Task A = taskflow.emplace([=]() { *(input.cnt) = 10; }).name("A");

  // create a subflow of two tasks B1->B2
  tf::Task B = taskflow
                   .emplace([=](tf::Subflow& subflow) {
                     std::vector<tf::Task> dynamic_tasks;
                     for (int i = 0; i < *(input.cnt); ++i)
                     {
                       tf::Task subB = subflow.emplace([]() {}).name("B" + std::to_string(i));
                       if (i != 0)
                         subB.succeed(dynamic_tasks.back());

                       dynamic_tasks.push_back(subB);
                     }
                   })
                   .name("B");

  A.precede(B);

  executor.run(taskflow).wait();  // run the taskflow to spawn subflows

  std::ofstream out_data;
  out_data.open(tesseract_common::getTempPath() + "subtaskflow_example-" + tesseract_common::getTimestampString() +
                ".dot");
  taskflow.dump(out_data);  // dump the graph including dynamic tasks
  out_data.close();
}
