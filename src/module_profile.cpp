
/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <rclcpp/rclcpp.hpp>

#define PROFILE_DATA_FILE "profile.dat"

#define MAX_PROFILES 128

#define NUM_PERF_COUNTERS 1

#define FOCUS_COUNTER 0
#define MAX_NUM_EVENTS 256
#define NUM_EVENT_COUNTERS 4
#include <boost/thread/lock_algorithms.hpp>
#include <boost/thread/locks.hpp>

#define PARSE_FUNCTION
#ifdef ENABLE_FUNCTION_PROFILING
static void ReadProfile(const char * prof_file_name);
static int CreateProfileParsing(const char * file_name, int ID);
static void CloseProfileParsing();
static void InitProfileParsing();
static int GetFreeProfile(int ID);
static int GetProfile(int ID);
static int AnalyzeProfile(const char * prof_file);
static void perf_event_init();
static void configure_events();

#include <string>

typedef struct ProfStructTag
{
  FILE * fp;
  int ID;
  std::string func_name;
  short start_end;
  double millisecs[NUM_PERF_COUNTERS];
  double total_millisecs[NUM_PERF_COUNTERS];
  double avg_millisecs[NUM_PERF_COUNTERS];
  double maximum_millisecs[NUM_PERF_COUNTERS];
  unsigned long long maximum_pos[NUM_PERF_COUNTERS];
  unsigned long long num_calls;

} prof_struct;
prof_struct profiles[MAX_PROFILES];

#include "module_profile.h"

FILE * prof_file;
int profile_count;
volatile int prof_buff[2048 * 100];
volatile int * pprof_buff;
FILE * fp_prof;

#ifdef DETAILED_PERF_ANALYSIS
int profile_event_start = 0;
int total_perf_events = 0;
#endif

int is_stream_end(void * fp);
int stream_close(void * fp);
int stream_write(char * p_buff, int size, int count, void * fp);
int stream_read(char * p_buff, int size, int count, void * fp);
void * stream_open(std::string name, std::string mode);

static int AnalyzeProfile(const char * prof_file)
{
  InitProfileParsing();
  ReadProfile(prof_file);
  CloseProfileParsing();
  return 0;
}

static void ReadProfile(const char * prof_file_name)
{
  int ID = 0;
  int profile_handle = 0;
  double millisecs[NUM_PERF_COUNTERS];
  double millisecs_spent;
  unsigned int two_pow_32 = (unsigned int)1 << 31;
  int is_maximum;
  int i;

  std::string prof_file_name1 = prof_file_name;

  prof_file = (FILE *)stream_open(prof_file_name1, "rb");

  if (prof_file == NULL) {
    printf("Could not open the file %s", prof_file_name);
  }

  while (!is_stream_end(prof_file)) {
    ID = -1;
    stream_read((char *)&ID, sizeof(int), 1, prof_file);
    profile_handle = GetProfile(ID);
    if ((ID <= -1) || (ID > MAX_PROFILES)) {
      continue;
    }
    if (profile_handle == -1) {
      profile_handle = CreateProfileParsing(prof_file_name, ID);

      if ((profile_handle <= -1) || (profile_handle >= MAX_PROFILES)) {
        printf("Could not create the profile file\n");
        continue;
      }
    }

    unsigned long long nanosecs_timestamp;
    stream_read(
      (char *)&nanosecs_timestamp, sizeof(nanosecs_timestamp), NUM_PERF_COUNTERS, prof_file);
    double s_part = ((int)(nanosecs_timestamp & 0xFFFFFFFF));
    double ns_part = ((int)(nanosecs_timestamp >> 32)) / 1000000000.0;

    millisecs[FOCUS_COUNTER] = (s_part + ns_part) * 1000;
    // std::cout << "Profile Id : " << profile_handle << " : " << millisecs[FOCUS_COUNTER] << std::endl;

    if (profiles[profile_handle].start_end == 1) {
      is_maximum = 0;

      for (i = 0; i < NUM_PERF_COUNTERS; i++) {
        if (i == 0) {
          millisecs_spent =
            millisecs[FOCUS_COUNTER] - profiles[profile_handle].millisecs[FOCUS_COUNTER];

          if (profiles[profile_handle].maximum_millisecs[FOCUS_COUNTER] < millisecs_spent) {
            is_maximum = 1;
          }
        }

        millisecs_spent = millisecs[i] - profiles[profile_handle].millisecs[i];

        profiles[profile_handle].total_millisecs[i] += millisecs_spent;

        if ((is_maximum) && (profiles[profile_handle].num_calls > 0)) {
          profiles[profile_handle].maximum_millisecs[i] = millisecs_spent;
          profiles[profile_handle].maximum_pos[i] = profiles[profile_handle].num_calls;
        }
      }
      profiles[profile_handle].num_calls++;
      profiles[profile_handle].start_end = 0;
    } else {
      for (i = 0; i < NUM_PERF_COUNTERS; i++) {
        profiles[profile_handle].millisecs[i] = millisecs[i];
      }

      profiles[profile_handle].start_end = 1;
    }
  }

  stream_close(prof_file);
}

static int GetFreeProfile(int ID)
{
  if (profile_count < ID) {
    profile_count = ID;
  }
  return (ID);
}

static void InitProfileParsing(void)
{
  int i, j;
  profile_count = 0;

  for (i = 0; i < MAX_PROFILES; i++) {
    profiles[i].fp = NULL;
    profiles[i].ID = -1;
    profiles[i].start_end = 0;
    for (j = 0; j < NUM_PERF_COUNTERS; j++) {
      profiles[i].millisecs[j] = 0;
      profiles[i].total_millisecs[j] = 0;
      profiles[i].avg_millisecs[j] = 0;
      profiles[i].maximum_millisecs[j] = 0;
    }

    profiles[i].num_calls = 0;
  }

  profile_id_init();
}

static int GetProfile(int ID)
{
  int i;

  for (i = 0; i < MAX_PROFILES; i++) {
    if (profiles[i].ID == ID) {
      return i;
    }
  }
  return -1;
}

static int CreateProfileParsing(const char * file_name, int ID)
{
  short profile_handle;
  char dest_prof_file[300];
  char str_ID[20];

  profile_handle = GetFreeProfile(ID);

  if (profile_handle == -1) {
    return -1;
  }

#ifdef DUMP_INDIVIDUAL_PROFILES
  sprintf(str_ID, "_%d", ID);
  strcpy(dest_prof_file, file_name);
  strcat(dest_prof_file, str_ID);

  profiles[profile_handle].fp = stream_open(dest_prof_file, "wb");
  profiles[profile_handle].ID = ID;

  if (profiles[profile_handle].fp == NULL) {
    return -1;
  }
#endif
  return profile_handle;
}

static void CloseProfileParsing(void)
{
  unsigned int handle, i, j;
  prof_struct temp;
  unsigned long long max_cycs_consumed;
  std::string ChannelString = "Millisecs";

  char out_msg[100];

#ifndef DETAILED_PERF_ANALYSIS
  fprintf(stdout, "\n\n%-40s", "Profile for function in milliseconds");
  fprintf(stdout, "%-12s", "Calls");
  fprintf(stdout, "%-15s", "Total(ms)");
  fprintf(stdout, "%-15s", "Avg(ms)");
  fprintf(stdout, "%-15s", "Max(ms)");
  // fprintf(stdout, "%-15s", "Avg MilliSecs/Pel");
  fprintf(stdout, "%-12s", "Max Pos");
  // fprintf(stdout, "%-15s", "Avg Time(ms)");
  for (i = 1; i < NUM_PERF_COUNTERS; i++) {
    sprintf(out_msg, "MAX %s", ChannelString.c_str());
    fprintf(stdout, "%-22s", out_msg);
  }

  fprintf(stdout, "%-15s\n", "Per Cent");
#endif

  for (i = 1; i <= profile_count; i++) {
    for (j = i; j <= profile_count; j++) {
      if (profiles[i].total_millisecs[0] < profiles[j].total_millisecs[0]) {
        temp = profiles[i];
        profiles[i] = profiles[j];
        profiles[j] = temp;
      }
    }
  }
  max_cycs_consumed = profiles[0].total_millisecs[0];
  for (handle = 0; handle <= profile_count; handle++) {
    if (profiles[handle].num_calls != 0) {
      profiles[handle].avg_millisecs[0] =
        profiles[handle].total_millisecs[0] / profiles[handle].num_calls;
#ifndef DETAILED_PERF_ANALYSIS
      fprintf(stdout, "%-40s", profiles[handle].func_name.c_str());
      fprintf(stdout, "%-12lld", profiles[handle].num_calls);
      fprintf(stdout, "%-15.4f", profiles[handle].total_millisecs[0]);
      fprintf(stdout, "%-15.4f", profiles[handle].avg_millisecs[0]);
      fprintf(stdout, "%-15.4f", profiles[handle].maximum_millisecs[0]);
      fprintf(stdout, "%-12lld", profiles[handle].maximum_pos[0]);
      // fprintf(stdout, "%-15f", (float)profiles[handle].avg_millisecs[0] / (float)(512 * 512));
      // double avg_time_ms = (double)profiles[handle].avg_millisecs[0];
      // fprintf(stdout, "%-15.4f", avg_time_ms);
#endif
      for (i = 1; i < NUM_PERF_COUNTERS; i++) {
#ifndef DETAILED_PERF_ANALYSIS
        sprintf(
          out_msg, "%f(%2.2f)", profiles[handle].maximum_millisecs[i],
          (float)profiles[handle].maximum_millisecs[i] * 100 /
            profiles[handle].maximum_millisecs[0]);
        fprintf(stdout, "%-22s", out_msg);
#else
        profiles[handle].event_millisecs[i + profile_event_start - 1] =
          profiles[handle].maximum_millisecs[i];
#endif
      }

#ifndef DETAILED_PERF_ANALYSIS
      fprintf(
        stdout, " %4.2f\n",
        ((float)profiles[handle].total_millisecs[0] / (float)max_cycs_consumed) * 100.0f);
#endif
#ifdef DUMP_INDIVIDUAL_PROFILES
      stream_close(profiles[handle].fp);
#endif
    }
  }
}

void initProfile(void)
{
  pprof_buff = &prof_buff[0];
  std::string PROFILE_DATA_FILE1 = PROFILE_DATA_FILE;
  fp_prof = (FILE *)stream_open(PROFILE_DATA_FILE1, "wb");

#ifdef DETAILED_PERF_ANALYSIS
  perf_event_init();
  configure_events();
#endif
}

std::recursive_mutex mtx;
void flushProfile(void)
{
  std::lock_guard<std::recursive_mutex> lock(mtx);
  int num_ints = ((pprof_buff - &prof_buff[0]) * sizeof(prof_buff[0]));

  if (num_ints > sizeof(prof_buff)) {
    printf("\nError: Profile buffer overflow, %d ints needed\n", num_ints);
  }

  stream_write((char *)&prof_buff[0], 1, num_ints, fp_prof);

  pprof_buff = &prof_buff[0];
}

void closeProfile()
{
  stream_close(fp_prof);
  std::string PROFILE_DATA_FILE1 = PROFILE_DATA_FILE;
  AnalyzeProfile(PROFILE_DATA_FILE1.c_str());
#ifdef DETAILED_PERF_ANALYSIS
  profile_event_start += NUM_EVENT_COUNTERS;
#endif
}

#ifndef DISABLE_FILE_IO
int is_stream_end(void * fp) { return (feof((FILE *)fp)); }

int stream_close(void * fp) { return (fclose((FILE *)fp)); }

int stream_write(char * p_buff, int size, int count, void * fp)
{
  while (count > 0) {
    fwrite(p_buff, size, 1, (FILE *)fp);
    p_buff++;
    count--;
  }
  return 0;
}

int stream_read(char * p_buff, int size, int count, void * fp)
{
  return (fread(p_buff, size, count, (FILE *)fp));
}
void * stream_open(std::string name, std::string mode)
{
  return (fopen(name.c_str(), mode.c_str()));
}
#else
int is_stream_end(void * fp)
{
  IO * p_io = (IO *)fp;

  return (p_io->data_available == 0);
}

int stream_close(void * fp) { return 0; }

int stream_write(char * p_buff, int size, int count, void * fp)
{
  IO * p_io = (IO *)fp;
  int num_bytes = count * size;

  if (num_bytes > p_io->space_available) {
    count = p_io->space_available / size;

    num_bytes = count * size;
  }

  p_io->space_available -= num_bytes;
  p_io->data_available += num_bytes;

  memcpy(&p_io->p_buf[p_io->curr_pos], p_buff, num_bytes);
  p_io->curr_pos += num_bytes;

  return count;
}

int stream_read(char * p_buff, int size, int count, void * fp)
{
  IO * p_io = (IO *)fp;
  int num_bytes = count * size;

  if (num_bytes > p_io->data_available) {
    count = p_io->data_available / size;

    num_bytes = count * size;
  }

  p_io->data_available -= num_bytes;
  memcpy(p_buff, &p_io->p_buf[p_io->curr_pos], num_bytes);
  p_io->curr_pos += num_bytes;

  return count;
}
void * stream_open(char * name, char * mode)
{
  IO * p_io = &io_struct;
  if (mode[0] == 'w') {
    p_io->data_available = 0;
    p_io->space_available = sizeof(io_buffer);
    p_io->p_buf = &io_buffer[0];
  }
  p_io->curr_pos = 0;

  return p_io;
}
#endif

#ifdef DETAILED_PERF_ANALYSIS
short perf_events[MAX_NUM_EVENTS];

static void perf_event_init(void)
{
  int i, event;

  /* Cortex A-8 events 0X13 - 0X3F are reserved */
  for (i = 0, event = 0; event <= 0x12; i++, event++) {
    perf_events[i] = event;
  }

  for (event = 0x40; event <= 0x5a; i++, event++) {
    perf_events[i] = event;
  }

  total_perf_events = i;

  printf("Total events = %d\n", total_perf_events);
}

static void configure_events(void)
{
  int i;
  int event_id;

  for (i = 0; i < NUM_EVENT_COUNTERS; i++) {
    disable_pmn(i);
    event_id = perf_events[i + profile_event_start];
    pmn_config(i, event_id);
    enable_pmn(i);
  }
}

int done_perf_analysis()
{
  int i, handle, done = 0;
  if (profile_event_start >= total_perf_events) {
    printf("Profiling Events Information\n");
    for (handle = 0; handle <= profile_count; handle++) {
      if (profiles[handle].num_calls != 0) {
        printf("%s\n", profiles[handle].func_name);

        for (i = 0; i < total_perf_events; i++) {
          printf("%4x:%d\n", perf_events[i], profiles[handle].event_millisecs[i]);
        }
      }
      printf("\n\n");
    }
    done = 1;
  }

  return done;
}

#endif
void dumpFunctionParams(int ID)
{
  std::lock_guard<std::recursive_mutex> lock(mtx);
  do {
    *pprof_buff++ = ID;
    rclcpp::Time curr_frame_timestamp_ = rclcpp::Clock{}.now();
    // rclcpp::Time curr_frame_timestamp_after;
    // std::cout << ID << " : "<< curr_frame_timestamp_ << std::endl;
    memcpy((void *)pprof_buff, &curr_frame_timestamp_, sizeof(curr_frame_timestamp_));
    // memcpy(&curr_frame_timestamp_after, (void *)pprof_buff,sizeof(curr_frame_timestamp_));
    // std::cout << "After : "<<ID << " : "<< curr_frame_timestamp_after << std::endl;
    pprof_buff += sizeof(curr_frame_timestamp_) / sizeof(pprof_buff[0]);
    // if (curr_frame_timestamp_after != curr_frame_timestamp_)
    //{
    //  std::cout << "Before and afer not matching....\n" <<std::endl << std::endl;
    //}
  } while (0);
}

#endif /* ENABLE_FUNCTION_PROFILING */
