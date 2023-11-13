#!/bin/bash

# Run commands concurrently
./run_graph_cmd --actor "Actor A" --actions-path . --graph-name engine_test_8 --sync-plugin action_sync_cyclone_dds &
pid1=$!

./run_graph_cmd --actor "Actor B" --actions-path . --graph-name engine_test_8 --sync-plugin action_sync_cyclone_dds &
pid2=$!

./run_graph_cmd --actor "Actor C" --actions-path . --graph-name engine_test_8 --sync-plugin action_sync_cyclone_dds &
pid3=$!

# Wait for both commands to finish
wait $pid1
status1=$?

wait $pid2
status2=$?

wait $pid3
status3=$?

# Print the return values
echo "Command 1 exit status: $status1"
echo "Command 2 exit status: $status2"
echo "Command 3 exit status: $status3"
