/**
 * @file jobEngine.h
 * @brief Interacts with the AWS Jobs API to pull any relevant jobs for this Thing, parse them and store the job with any relevant info
 *          inside a struct that needs to be dealt with by the rest of the application
 * @author Jack Cripps // jackc@monitoraudio.com
 * @date 11/09/2019
 */

#ifndef __JOB_AGENT_H__
#define __JOB_AGENT_H__


/* Function Prototypes
 * -------------------------------------------------------------------------- */

void job_agent_task(void *param);



#endif /* __JOB_AGENT_H__ */
