This contains two packages:
 - `simple_timer` contains timing related functions, and a wrapper for calling some kernel scheduling functions
 - `priority_executor` contains the modified executor and some nodes to test it

In `priority_executor`:
 - `priority_executor.cpp` subclasses `rclcpp::Executor` to allow for additional customization
 - `priority_memory_strategy.hpp` is a modified version of `rclcpp`s `allocator_memory_strategy.hpp` that selects callbacks based on either the earliest deadline, or a relative priority. Executor polls this for ready callbacks. 
 - `test_nodes.cpp` adds timer-based publishing nodes and dummy worker nodes that can be arranged in chains. It uses `dummy_workload.hpp` to generate a workload.
 - `f1tenth_test.cpp` sets up a chain of nodes similar to [https://intra.ece.ucr.edu/~hyoseung/pdf/rtas21_picas.pdf](https://intra.ece.ucr.edu/~hyoseung/pdf/rtas21_picas.pdf)

