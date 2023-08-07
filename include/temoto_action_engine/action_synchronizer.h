/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Copyright 2023 TeMoto Framework
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef TEMOTO_ACTION_ENGINE__ACTION_SYNCHRONIZER_H
#define TEMOTO_ACTION_ENGINE__ACTION_SYNCHRONIZER_H

#include "temoto_action_engine/waitlist.h"

class ActionSynchronizer
{
public:

  virtual void notify(Waitable waitable, unsigned int ack_count) = 0;

  virtual void acknowledge(Waitable waitable, Waiter waiter) = 0;

  virtual void setNotifyCallback(NotifyFinishedT notify_finished_fptr_) = 0;

};

#endif