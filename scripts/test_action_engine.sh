#!/bin/bash

RED="\e[31m"
YELLOW="\e[33m"
GREEN="\e[32m"
BOLD="\e[1m"
NL="\n"
RESET="\e[39m\e[0m"

NR_OF_TEST_RUNS=30;
SUCCESSFUL_TEST_RUNS=0

echo -e $YELLOW$BOLD"Running a thread test for $NR_OF_TEST_RUNS times" $RESET

for (( i=0; i < $NR_OF_TEST_RUNS; i++ )); do
  echo -e -n "Running test nr $i:" $RESET

  SECONDS=0
  rosrun temoto_action_engine temoto_ae_test > /dev/null 2>&1
  EXIT_CODE=$?
  duration=$SECONDS

  # Check if the run was successful or 
  if [[ $EXIT_CODE != 3 ]]; then
    echo -e -n $RED$BOLD"Fail."$RESET
  else
    echo -e -n $GREEN"Success."$RESET
    SUCCESSFUL_TEST_RUNS=$((SUCCESSFUL_TEST_RUNS + 1))
  fi

  echo " Test elapsed $(($duration / 60))m and $(($duration % 60))s."
done

echo -e $YELLOW$BOLD"$SUCCESSFUL_TEST_RUNS tests out of $NR_OF_TEST_RUNS were successful" $RESET
