/**
 * @file jobEngine.c
 * @brief Interacts with the AWS Jobs API to pull any relevant jobs for this Thing, parse them and store the job with any relevant info
 *          inside a struct, which is then placed into a buffer of jobs that need to be dealt with by the rest of the application
 * @author Jack Cripps // jackc@monitoraudio.com
 * @date 11/09/2019
 */

#include "ma_jobAgent.h"

#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

