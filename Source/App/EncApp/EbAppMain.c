/*
* Copyright(c) 2019 Intel Corporation
* SPDX - License - Identifier: BSD - 2 - Clause - Patent
*/

// main.cpp
//  -Contructs the following resources needed during the encoding process
//      -memory
//      -threads
//      -semaphores
//      -semaphores
//  -Configures the encoder
//  -Calls the encoder via the API
//  -Destructs the resources

/***************************************
 * Includes
 ***************************************/
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <stdint.h>
#include "EbAppConfig.h"
#include "EbAppContext.h"
#include "EbSvtAv1Time.h"
#ifdef _WIN32
#include <Windows.h>
#else
#include <pthread.h>
#include <semaphore.h>
#include <time.h>
#include <errno.h>
#endif

#ifdef _MSC_VER
#include <io.h>     /* _setmode() */
#include <fcntl.h>  /* _O_BINARY */
#endif
/***************************************
 * External Functions
 ***************************************/
extern AppExitConditionType ProcessInputBuffer(
    EbConfig             *config,
    EbAppContext         *appCallBack);

extern AppExitConditionType ProcessOutputReconBuffer(
    EbConfig             *config,
    EbAppContext         *appCallBack);

extern AppExitConditionType ProcessOutputStreamBuffer(
    EbConfig             *config,
    EbAppContext         *appCallBack,
    uint8_t           pic_send_done);

volatile int32_t keepRunning = 1;

void EventHandler(int32_t dummy) {
    (void)dummy;
    keepRunning = 0;

    // restore default signal handler
    signal(SIGINT, SIG_DFL);
}

void AssignAppThreadGroup(uint8_t target_socket) {
#ifdef _MSC_VER
    if (GetActiveProcessorGroupCount() == 2) {
        GROUP_AFFINITY           group_affinity;
        GetThreadGroupAffinity(GetCurrentThread(), &group_affinity);
        group_affinity.Group = target_socket;
        SetThreadGroupAffinity(GetCurrentThread(), &group_affinity, NULL);
    }
#else
    (void)target_socket;
    return;
#endif
}

/***************************************
 * Encoder App Main
 ***************************************/
int32_t main(int32_t argc, char* argv[])
{
#ifdef _MSC_VER
    _setmode(_fileno(stdin), _O_BINARY);
    _setmode(_fileno(stdout), _O_BINARY);
#endif
    // GLOBAL VARIABLES
    EbErrorType            return_error = EB_ErrorNone;            // Error Handling
    AppExitConditionType    exitCondition = APP_ExitConditionNone;    // Processing loop exit condition

    EbErrorType            return_errors[MAX_CHANNEL_NUMBER];          // Error Handling
    AppExitConditionType    exitConditions[MAX_CHANNEL_NUMBER];          // Processing loop exit condition
    AppExitConditionType    exitConditionsOutput[MAX_CHANNEL_NUMBER];         // Processing loop exit condition
    AppExitConditionType    exitConditionsRecon[MAX_CHANNEL_NUMBER];         // Processing loop exit condition
    AppExitConditionType    exitConditionsInput[MAX_CHANNEL_NUMBER];          // Processing loop exit condition


    EbBool                 channelActive[MAX_CHANNEL_NUMBER];

    EbConfig             *configs[MAX_CHANNEL_NUMBER];        // Encoder Configuration

    uint32_t                num_channels = 0;
    uint32_t                instanceCount=0;
    EbAppContext         *appCallbacks[MAX_CHANNEL_NUMBER];   // Instances App callback data
    signal(SIGINT, EventHandler);
    printf("-------------------------------------------\n");
    printf("SVT-AV1 Encoder\n");
    if (!get_help(argc, argv)) {

        // Get num_channels
        num_channels = get_number_of_channels(argc, argv);
        if (num_channels == 0) {
            return EB_ErrorBadParameter;
        }

        // Initialize config
        for (instanceCount = 0; instanceCount < num_channels; ++instanceCount) {
            configs[instanceCount] = (EbConfig*)malloc(sizeof(EbConfig));
            if (!configs[instanceCount])
                return EB_ErrorInsufficientResources;
            eb_config_ctor(configs[instanceCount]);
            return_errors[instanceCount] = EB_ErrorNone;
        }

        // Initialize appCallback
        for (instanceCount = 0; instanceCount < num_channels; ++instanceCount) {
            appCallbacks[instanceCount] = (EbAppContext*)malloc(sizeof(EbAppContext));
            if (!appCallbacks[instanceCount])
                return EB_ErrorInsufficientResources;
        }

        for (instanceCount = 0; instanceCount < MAX_CHANNEL_NUMBER; ++instanceCount) {
            exitConditions[instanceCount] = APP_ExitConditionError;         // Processing loop exit condition
            exitConditionsOutput[instanceCount] = APP_ExitConditionError;         // Processing loop exit condition
            exitConditionsRecon[instanceCount] = APP_ExitConditionError;         // Processing loop exit condition
            exitConditionsInput[instanceCount] = APP_ExitConditionError;         // Processing loop exit condition
            channelActive[instanceCount] = EB_FALSE;
        }

        // Read all configuration files.
        return_error = read_command_line(argc, argv, configs, num_channels, return_errors);

        // Process any command line options, including the configuration file

        if (return_error == EB_ErrorNone) {

            // Set main thread affinity
            if (configs[0]->target_socket != -1)
                AssignAppThreadGroup(configs[0]->target_socket);

            // Init the Encoder
            for (instanceCount = 0; instanceCount < num_channels; ++instanceCount) {
                if (return_errors[instanceCount] == EB_ErrorNone) {

                    configs[instanceCount]->active_channel_count = num_channels;
                    configs[instanceCount]->channel_id = instanceCount;

                    EbStartTime((uint64_t*)&configs[instanceCount]->performance_context.lib_start_time[0], (uint64_t*)&configs[instanceCount]->performance_context.lib_start_time[1]);

                    return_errors[instanceCount] = init_encoder(configs[instanceCount], appCallbacks[instanceCount], instanceCount);
                    return_error = (EbErrorType)(return_error | return_errors[instanceCount]);
                }
                else {
                    channelActive[instanceCount] = EB_FALSE;
                }
            }

            {
                // Start the Encoder
                for (instanceCount = 0; instanceCount < num_channels; ++instanceCount) {
                    if (return_errors[instanceCount] == EB_ErrorNone) {
                        return_error = (EbErrorType)(return_error & return_errors[instanceCount]);
                        exitConditions[instanceCount]       = APP_ExitConditionNone;
                        exitConditionsOutput[instanceCount] = APP_ExitConditionNone;
                        exitConditionsRecon[instanceCount]  = configs[instanceCount]->recon_file ? APP_ExitConditionNone : APP_ExitConditionError;
                        exitConditionsInput[instanceCount]  = APP_ExitConditionNone;
                        channelActive[instanceCount]        = EB_TRUE;
                        EbStartTime((uint64_t*)&configs[instanceCount]->performance_context.encode_start_time[0], (uint64_t*)&configs[instanceCount]->performance_context.encode_start_time[1]);

                    }
                    else {
                        exitConditions[instanceCount]       = APP_ExitConditionError;
                        exitConditionsOutput[instanceCount] = APP_ExitConditionError;
                        exitConditionsRecon[instanceCount]  = APP_ExitConditionError;
                        exitConditionsInput[instanceCount]  = APP_ExitConditionError;
                    }

#if DISPLAY_MEMORY
                    EB_APP_MEMORY();
#endif
                }
                printf("Encoding          ");
                fflush(stdout);

                while (exitCondition == APP_ExitConditionNone) {
                    exitCondition = APP_ExitConditionFinished;
                    for (instanceCount = 0; instanceCount < num_channels; ++instanceCount) {
                        if (channelActive[instanceCount] == EB_TRUE) {
                            if (exitConditionsInput[instanceCount] == APP_ExitConditionNone)
                                exitConditionsInput[instanceCount] = ProcessInputBuffer(
                                                                            configs[instanceCount],
                                                                            appCallbacks[instanceCount]);
                            if (exitConditionsRecon[instanceCount] == APP_ExitConditionNone)
                                exitConditionsRecon[instanceCount] = ProcessOutputReconBuffer(
                                                                            configs[instanceCount],
                                                                            appCallbacks[instanceCount]);
                            if (exitConditionsOutput[instanceCount] == APP_ExitConditionNone)
                                exitConditionsOutput[instanceCount] = ProcessOutputStreamBuffer(
                                                                            configs[instanceCount],
                                                                            appCallbacks[instanceCount],
                                                                            (exitConditionsInput[instanceCount] == APP_ExitConditionNone) || (exitConditionsRecon[instanceCount] == APP_ExitConditionNone)? 0 : 1);
                            if (((exitConditionsRecon[instanceCount] == APP_ExitConditionFinished || !configs[instanceCount]->recon_file)  && exitConditionsOutput[instanceCount] == APP_ExitConditionFinished && exitConditionsInput[instanceCount] == APP_ExitConditionFinished)||
                                ((exitConditionsRecon[instanceCount] == APP_ExitConditionError && configs[instanceCount]->recon_file) || exitConditionsOutput[instanceCount] == APP_ExitConditionError || exitConditionsInput[instanceCount] == APP_ExitConditionError)){
                                channelActive[instanceCount] = EB_FALSE;
                                if (configs[instanceCount]->recon_file)
                                    exitConditions[instanceCount] = (AppExitConditionType)(exitConditionsRecon[instanceCount] | exitConditionsOutput[instanceCount] | exitConditionsInput[instanceCount]);
                                else
                                    exitConditions[instanceCount] = (AppExitConditionType)(exitConditionsOutput[instanceCount] | exitConditionsInput[instanceCount]);
                            }
                        }
                    }
                    // check if all channels are inactive
                    for (instanceCount = 0; instanceCount < num_channels; ++instanceCount) {
                        if (channelActive[instanceCount] == EB_TRUE)
                            exitCondition = APP_ExitConditionNone;
                    }
                }

                for (instanceCount = 0; instanceCount < num_channels; ++instanceCount) {
                    if (exitConditions[instanceCount] == APP_ExitConditionFinished && return_errors[instanceCount] == EB_ErrorNone) {
                        double frame_rate;

                        if ((configs[instanceCount]->frame_rate_numerator != 0 && configs[instanceCount]->frame_rate_denominator != 0) || configs[instanceCount]->frame_rate != 0) {

                            if (configs[instanceCount]->frame_rate_numerator && configs[instanceCount]->frame_rate_denominator && (configs[instanceCount]->frame_rate_numerator != 0 && configs[instanceCount]->frame_rate_denominator != 0)) {
                                frame_rate = ((double)configs[instanceCount]->frame_rate_numerator) / ((double)configs[instanceCount]->frame_rate_denominator);
                            }
                            else if (configs[instanceCount]->frame_rate > 1000) {
                                // Correct for 16-bit fixed-point fractional precision
                                frame_rate = ((double)configs[instanceCount]->frame_rate) / (1 << 16);
                            }
                            else {
                                frame_rate = (double)configs[instanceCount]->frame_rate;
                            }
                            printf("\nSUMMARY --------------------------------- Channel %u  --------------------------------\n", instanceCount + 1);

                            // Interlaced Video
                            if (configs[instanceCount]->interlaced_video || configs[instanceCount]->separate_fields) {
                                printf("Total Fields\t\tFrame Rate\t\tByte Count\t\tBitrate\n");
                            }
                            else {
                                printf("Total Frames\t\tFrame Rate\t\tByte Count\t\tBitrate\n");
                            }
                            printf("%12d\t\t%4.2f fps\t\t%10.0f\t\t%5.2f kbps\n",
                                (int32_t)configs[instanceCount]->performance_context.frame_count,
                                (double)frame_rate,
                                (double)configs[instanceCount]->performance_context.byte_count,
                                ((double)(configs[instanceCount]->performance_context.byte_count << 3) * frame_rate / (configs[instanceCount]->frames_encoded * 1000)));
                            fflush(stdout);
                        }
                    }
                }
                printf("\n");
                fflush(stdout);
            }
            for (instanceCount = 0; instanceCount < num_channels; ++instanceCount) {
                if (exitConditions[instanceCount] == APP_ExitConditionFinished && return_errors[instanceCount] == EB_ErrorNone) {

                    if (configs[instanceCount]->stop_encoder == EB_FALSE) {
                        // Interlaced Video
                        if (configs[instanceCount]->interlaced_video || configs[instanceCount]->separate_fields) {

                            printf("\nChannel %u\nAverage Speed:\t\t%.0f fields per sec\nTotal Encoding Time:\t\t%.0f ms\nTotal Execution Time:\t\t%.2f ms\nAverage Latency:\t%.0f ms\nMax Latency:\t\t%u ms\n",
                                (uint32_t)(instanceCount + 1),
                                configs[instanceCount]->performance_context.average_speed,
                                configs[instanceCount]->performance_context.total_encode_time * 1000,
                                configs[instanceCount]->performance_context.total_execution_time * 1000,
                                configs[instanceCount]->performance_context.average_latency,
                                (uint32_t)(configs[instanceCount]->performance_context.max_latency));
                        }
                        else {
                            printf("\nChannel %u\nAverage Speed:\t\t%.3f fps\nTotal Encoding Time:\t%.0f ms\nTotal Execution Time:\t%.0f ms\nAverage Latency:\t%.0f ms\nMax Latency:\t\t%u ms\n",
                                (uint32_t)(instanceCount + 1),
                                configs[instanceCount]->performance_context.average_speed,
                                configs[instanceCount]->performance_context.total_encode_time * 1000,
                                configs[instanceCount]->performance_context.total_execution_time * 1000,
                                configs[instanceCount]->performance_context.average_latency,
                                (uint32_t)(configs[instanceCount]->performance_context.max_latency));

                        }
                    }
                    else {
                        printf("\nChannel %u Encoding Interrupted\n", (uint32_t)(instanceCount + 1));
                    }
                }
                else if (return_errors[instanceCount] == EB_ErrorInsufficientResources) {
                    printf("Could not allocate enough memory for channel %u\n", instanceCount + 1);
                }
                else {
                    printf("Error encoding at channel %u! Check error log file for more details ... \n", instanceCount + 1);
                }
            }
#if !CHECK_MEM_REDUCTION
            // DeInit Encoder
            for (instanceCount = num_channels; instanceCount > 0; --instanceCount) {
                if (return_errors[instanceCount - 1] == EB_ErrorNone)
                    return_errors[instanceCount - 1] = de_init_encoder(appCallbacks[instanceCount - 1], instanceCount - 1);
            }
#endif
        }
        else {
            printf("Error in configuration, could not begin encoding! ... \n");
            printf("Run %s -help for a list of options\n", argv[0]);
        }
        // Destruct the App memory variables
        for (instanceCount = 0; instanceCount < num_channels; ++instanceCount) {
            eb_config_dtor(configs[instanceCount]);
            if (configs[instanceCount])
                free(configs[instanceCount]);
            if (appCallbacks[instanceCount])
                free(appCallbacks[instanceCount]);
        }

        printf("Encoder finished\n");
    }

    return (return_error == 0) ? 0 : 1;
}

