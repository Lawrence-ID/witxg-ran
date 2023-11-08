/*
 * Licensed to the OpenAirInterface (OAI) Software Alliance under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The OpenAirInterface Software Alliance licenses this file to You under
 * the OAI Public License, Version 1.1  (the "License"); you may not use this file
 * except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.openairinterface.org/?page_id=698
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *-------------------------------------------------------------------------------
 * For more information about the OpenAirInterface (OAI) Software Alliance:
 *      contact@openairinterface.org
 */

/* Header file generated by fdesign on Thu Aug 28 12:13:51 2014 */

#ifndef _TARGETS_RT_USER_STATS_H_
#define _TARGETS_RT_USER_STATS_H_

#ifdef XFORMS
#include <forms.h>
#endif

/* Callbacks, globals and object handlers */

extern void reset_stats( FL_OBJECT *, long );

/* Forms and Objects */

typedef struct {
  FL_FORM   * stats_form;
  void      * vdata;
  char      * cdata;
  long        ldata;
  FL_OBJECT * stats_text;
  FL_OBJECT * stats_button;
} FD_stats_form;

extern FD_stats_form * create_form_stats_form( void );

#endif /* _TARGETS_RT_USER_STATS_H_ */
