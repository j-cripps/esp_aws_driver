/**
 * @file ma_jobParser.h
 * @brief Contains all details of the job parser engine
 * @author Jack Cripps // jackc@monitoraudio.com
 * @date 21/10/2019
 */

#ifndef __JOB_PARSER_H__
#define __JOB_PARSER_H__


static void jobQueryParser(jsmn_parser *pJsmnParser, char *topicName, uint16_t topicNameLen, IoT_Publish_Message_Params *params, void *pData);
static void jobDescriptionParser();
static void jobExecutionParser();


#endif /*__JOB_PARSER_H__*/
