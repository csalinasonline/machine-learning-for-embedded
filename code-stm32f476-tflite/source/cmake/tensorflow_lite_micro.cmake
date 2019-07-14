set(TENSORFLOW_LITE_DIR ${CMAKE_SOURCE_DIR}/libs/tensorflow)

# Make sure that git submodule is initialized and updated
if (NOT EXISTS "${TENSORFLOW_LITE_DIR}")
  message(FATAL_ERROR "Tensorflow-lite submodule not found. Initialize with 'git submodule update --init' in the source directory")
endif()

set (TENSORFLOW_LITE_INC
    ${TENSORFLOW_LITE_DIR}/lite/experimental/micro/kernels/
    ${TENSORFLOW_LITE_DIR}/lite/experimental/micro
    ${TENSORFLOW_LITE_DIR}/lite/kernels/internal/reference
    ${TENSORFLOW_LITE_DIR}/lite/kernels/internal
    ${TENSORFLOW_LITE_DIR}/lite/kernels
    ${TENSORFLOW_LITE_DIR}/lite/schema
    ${TENSORFLOW_LITE_DIR}/lite/core/api
    ${TENSORFLOW_LITE_DIR}/lite/core/c_api_internal
    ${TENSORFLOW_LITE_DIR}/lite/c
    ${TENSORFLOW_LITE_DIR}/lite
    ${TENSORFLOW_LITE_DIR}/lite/flatbuffers/include/flatbuffers
    ${TENSORFLOW_LITE_DIR}/lite/fixedpoint/internal
    ${TENSORFLOW_LITE_DIR}/lite/fixedpoint/fixedpoint
)

include_directories(
    ${TENSORFLOW_LITE_INC}
)

aux_source_directory(${TENSORFLOW_LITE_DIR}/lite/flatbuffers/src FLATBUFFERS_SRC)

# Get all source files from the Src directory
set(TENSORFLOW_LITE_SRC
    ${TENSORFLOW_LITE_DIR}/lite/experimental/micro/micro_mutable_op_resolver.cc
    ${TENSORFLOW_LITE_DIR}/lite/experimental/micro/kernels/depthwise_conv.cc
    ${TENSORFLOW_LITE_DIR}/lite/experimental/micro/kernels/softmax.cc
    ${TENSORFLOW_LITE_DIR}/lite/experimental/micro/kernels/all_ops_resolver.cc
    ${TENSORFLOW_LITE_DIR}/lite/experimental/micro/kernels/fully_connected.cc
    ${TENSORFLOW_LITE_DIR}/lite/experimental/micro/debug_log.cc
    ${TENSORFLOW_LITE_DIR}/lite/experimental/micro/debug_log_numbers.cc
    ${TENSORFLOW_LITE_DIR}/lite/experimental/micro/micro_error_reporter.cc
    ${TENSORFLOW_LITE_DIR}/lite/experimental/micro/micro_interpreter.cc
    ${TENSORFLOW_LITE_DIR}/lite/experimental/micro/simple_tensor_allocator.cc
    ${TENSORFLOW_LITE_DIR}/lite/kernels/kernel_util.cc
    ${TENSORFLOW_LITE_DIR}/lite/kernels/internal/quantization_util.cc
    ${TENSORFLOW_LITE_DIR}/lite/core/api/op_resolver.cc
    ${TENSORFLOW_LITE_DIR}/lite/core/api/flatbuffer_conversions.cc
    ${TENSORFLOW_LITE_DIR}/lite/core/api/error_reporter.cc
    ${TENSORFLOW_LITE_DIR}/lite/c/c_api_internal.c
    ${FLATBUFFERS_SRC}
)

# set_source_files_properties(${TENSORFLOW_LITE_SRC}
#     PROPERTIES COMPILE_FLAGS "${STM32F7_DEFINES} -Wno-fpermissive"
# )



add_library(Tensorflow_lite_micro STATIC ${TENSORFLOW_LITE_SRC})

set_target_properties(Tensorflow_lite_micro PROPERTIES LINKER_LANGUAGE CXX)

set(EXTERNAL_EXECUTABLES ${EXTERNAL_EXECUTABLES} ${STARTUP_ASM_FILE})

set(EXTERNAL_LIBS ${EXTERNAL_LIBS} Tensorflow_lite_micro)
