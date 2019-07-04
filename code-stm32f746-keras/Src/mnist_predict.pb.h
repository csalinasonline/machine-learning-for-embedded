/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.0-dev */

#ifndef PB_MNIST_PREDICT_PB_H_INCLUDED
#define PB_MNIST_PREDICT_PB_H_INCLUDED
#include <pb.h>

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef PB_BYTES_ARRAY_T(1024) MnistDigitPredictRequest_buffer_t;
typedef struct _MnistDigitPredictRequest {
    MnistDigitPredictRequest_buffer_t buffer;
/* @@protoc_insertion_point(struct:MnistDigitPredictRequest) */
} MnistDigitPredictRequest;


typedef PB_BYTES_ARRAY_T(100) MnistDigitPredictResponse_resp_t;
typedef struct _MnistDigitPredictResponse {
    uint32_t found_digit;
    uint32_t exec_time;
    MnistDigitPredictResponse_resp_t resp;
/* @@protoc_insertion_point(struct:MnistDigitPredictResponse) */
} MnistDigitPredictResponse;


/* Initializer values for message structs */
#define MnistDigitPredictRequest_init_default    {{0, {0}}}
#define MnistDigitPredictResponse_init_default   {0, 0, {0, {0}}}
#define MnistDigitPredictRequest_init_zero       {{0, {0}}}
#define MnistDigitPredictResponse_init_zero      {0, 0, {0, {0}}}

/* Field tags (for use in manual encoding/decoding) */
#define MnistDigitPredictRequest_buffer_tag      1
#define MnistDigitPredictResponse_found_digit_tag 1
#define MnistDigitPredictResponse_exec_time_tag  2
#define MnistDigitPredictResponse_resp_tag       3

/* Struct field encoding specification for nanopb */
#define MnistDigitPredictRequest_FIELDLIST(X, a) \
X(a, STATIC, REQUIRED, BYTES, buffer, 1)
#define MnistDigitPredictRequest_CALLBACK NULL
#define MnistDigitPredictRequest_DEFAULT NULL

#define MnistDigitPredictResponse_FIELDLIST(X, a) \
X(a, STATIC, REQUIRED, UINT32, found_digit, 1) \
X(a, STATIC, REQUIRED, UINT32, exec_time, 2) \
X(a, STATIC, REQUIRED, BYTES, resp, 3)
#define MnistDigitPredictResponse_CALLBACK NULL
#define MnistDigitPredictResponse_DEFAULT NULL

extern const pb_msgdesc_t MnistDigitPredictRequest_msg;
extern const pb_msgdesc_t MnistDigitPredictResponse_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define MnistDigitPredictRequest_fields &MnistDigitPredictRequest_msg
#define MnistDigitPredictResponse_fields &MnistDigitPredictResponse_msg

/* Maximum encoded size of messages (where known) */
#define MnistDigitPredictRequest_size            1027
#define MnistDigitPredictResponse_size           114

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
