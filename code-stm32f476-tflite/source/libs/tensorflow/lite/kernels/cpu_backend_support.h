/* Copyright 2019 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/
#ifndef TENSORFLOW_LITE_KERNELS_CPU_BACKEND_SUPPORT_H_
#define TENSORFLOW_LITE_KERNELS_CPU_BACKEND_SUPPORT_H_

#include "c_api_internal.h"
#include "cpu_backend_context.h"

namespace tflite {

namespace cpu_backend_support {

CpuBackendContext* GetFromContext(TfLiteContext* context);

void IncrementUsageCounter(TfLiteContext* context);

void DecrementUsageCounter(TfLiteContext* context);

}  // namespace cpu_backend_support
}  // namespace tflite

#endif  // TENSORFLOW_LITE_KERNELS_CPU_BACKEND_SUPPORT_H_
