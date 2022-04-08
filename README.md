This contains the priority_executor package that adds an Executor with deadline and chain-aware priority support.
 - `simple_timer` contains timing related functions, and a wrapper for calling some kernel scheduling functions
 - `priority_executor` contains the modified executor and some nodes to test it

In `priority_executor`:
 - `priority_executor.cpp` subclasses `rclcpp::Executor` to allow for additional customization
 - `priority_memory_strategy.hpp` is a modified version of `rclcpp`s `allocator_memory_strategy.hpp` that selects callbacks based on either the earliest deadline, or a relative priority. Executor polls this for ready callbacks. 
 - `usage_example.cpp` shows how to set deadlines for callbacks and timers

the `main` branch of this repository has the test scripts and nodes used in the paper.