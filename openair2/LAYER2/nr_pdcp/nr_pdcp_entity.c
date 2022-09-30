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

#include "nr_pdcp_entity.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "nr_pdcp_security_nea2.h"
#include "nr_pdcp_integrity_nia2.h"
#include "nr_pdcp_sdu.h"

#include "LOG/log.h"

static void nr_pdcp_entity_recv_pdu(nr_pdcp_entity_t *entity,
                                    char *_buffer, int size)
{
  unsigned char    *buffer = (unsigned char *)_buffer;
  nr_pdcp_sdu_t    *cur_sdu;
  int              rcvd_sn;
  uint32_t         rcvd_hfn;
  uint32_t         rcvd_count;
  int              header_size;
  int              integrity_size;
  int              rx_deliv_sn;
  uint32_t         rx_deliv_hfn;

  if (size < 1) {
    LOG_E(PDCP, "bad PDU received (size = %d)\n", size);
    return;
  }

  if (entity->type != NR_PDCP_SRB && !(buffer[0] & 0x80)) {
    LOG_E(PDCP, "%s:%d:%s: fatal\n", __FILE__, __LINE__, __FUNCTION__);
    /* TODO: This is something of a hack. The most significant bit
       in buffer[0] should be 1 if the packet is a data packet. We are
       processing malformed data packets if the most significant bit
       is 0. Rather than exit(1), this hack allows us to continue for now.
       We need to investigate why this hack is neccessary. */
    buffer[0] |= 128;
  }

  if (entity->sn_size == 12) {
    rcvd_sn = ((buffer[0] & 0xf) <<  8) |
                buffer[1];
    header_size = 2;
  } else {
    rcvd_sn = ((buffer[0] & 0x3) << 16) |
               (buffer[1]        <<  8) |
                buffer[2];
    header_size = 3;
  }

  /* SRBs always have MAC-I, even if integrity is not active */
  if (entity->has_integrity || entity->type == NR_PDCP_SRB) {
    integrity_size = 4;
  } else {
    integrity_size = 0;
  }

  if (size < header_size + integrity_size + 1) {
    LOG_E(PDCP, "bad PDU received (size = %d)\n", size);
    return;
  }

  rx_deliv_sn  = entity->rx_deliv & entity->sn_max;
  rx_deliv_hfn = entity->rx_deliv >> entity->sn_size;

  if (rcvd_sn < rx_deliv_sn - entity->window_size) {
    rcvd_hfn = rx_deliv_hfn + 1;
  } else if (rcvd_sn >= rx_deliv_sn + entity->window_size) {
    rcvd_hfn = rx_deliv_hfn - 1;
  } else {
    rcvd_hfn = rx_deliv_hfn;
  }

  rcvd_count = (rcvd_hfn << entity->sn_size) | rcvd_sn;

  if (entity->has_ciphering)
    entity->cipher(entity->security_context,
                   buffer+header_size, size-header_size,
                   entity->rb_id, rcvd_count, entity->is_gnb ? 0 : 1);

  if (entity->has_integrity) {
    unsigned char integrity[4];
    entity->integrity(entity->integrity_context, integrity,
                      buffer, size - integrity_size,
                      entity->rb_id, rcvd_count, entity->is_gnb ? 0 : 1);
    if (memcmp(integrity, buffer + size - integrity_size, 4) != 0) {
      LOG_E(PDCP, "discard NR PDU, integrity failed\n");
//      return;
    }
  }


//entity->rx_deliv is next to deliver
  if (rcvd_count < entity->rx_deliv)
  {
    LOG_W(PDCP, "discard NR PDU rcvd_count=%d, already deliver! \n", rcvd_count);
    return;
  }


  if (rcvd_count - entity->rx_deliv >= PDCP_RECV_WINDOW_MAX_LEN)
  {
    //if max=100, rx_deliv=5,rcvd_count=108, 108 need put in 8, so 
    //step1: need to deliver 5,6,7,8, rx_deliv and window_start_index should be 8.
    //step2: check from 9.
    //step3: use normal procedure: store 108, and check whether to deliver
    
    int num_need_to_delivery = rcvd_count - entity->rx_deliv - PDCP_RECV_WINDOW_MAX_LEN + 1; //be 4
    LOG_W(PDCP, "%d recv, %d wait to recv, too many in window, deliver %d in window! \n", rcvd_count,entity->rx_deliv,num_need_to_delivery);

    //step1: deliver 5,6,7,8 here.
    int real_deliver_num = 0;
    for(uint32_t i=entity->window_start_index;i<entity->window_start_index+num_need_to_delivery;i++)
    {
        cur_sdu = entity->pdcp_recv_window[i % PDCP_RECV_WINDOW_MAX_LEN];
        LOG_D(PDCP, "in step1: i %d, index %d, cur_sdu %p \n",i,i % PDCP_RECV_WINDOW_MAX_LEN,cur_sdu);
        if (cur_sdu != NULL)
        {
          entity->deliver_sdu(entity->deliver_sdu_data, entity,
                              cur_sdu->buffer, cur_sdu->size);
          nr_pdcp_free_sdu(cur_sdu);
          entity->pdcp_recv_window[i % PDCP_RECV_WINDOW_MAX_LEN] = NULL;
          real_deliver_num++;
        }
    }
    entity->rx_deliv = entity->rx_deliv + num_need_to_delivery; 
    entity->window_start_index = (entity->window_start_index + num_need_to_delivery) % PDCP_RECV_WINDOW_MAX_LEN;
    entity->pdcp_recv_in_window_count -= real_deliver_num;

    LOG_D(PDCP, "step1: rx_deliv %d, window_start_index %d, real_deliver_num %d, pdcp_recv_in_window_count %d\n", 
          entity->rx_deliv,entity->window_start_index,real_deliver_num,entity->pdcp_recv_in_window_count);

    //step2: 
    // after deliver 5,6,7,8; 8 is free for 108, and 108 will store in 8 later.
    // rx_deliv and  window_start_index should be for 9, and check from 9.
    int check_next_index = entity->window_start_index;
    cur_sdu = entity->pdcp_recv_window[check_next_index];
    real_deliver_num = 0;
    while(cur_sdu != NULL)
    {
      LOG_D(PDCP, "in step2: check_next_index %d, real_deliver_num %d\n", 
          check_next_index,real_deliver_num);
      entity->deliver_sdu(entity->deliver_sdu_data, entity,
                            cur_sdu->buffer, cur_sdu->size);
      //printf("%d \n",10/real_deliver_num);
      nr_pdcp_free_sdu(cur_sdu);
      entity->pdcp_recv_window[check_next_index]= NULL;
      real_deliver_num++;
      check_next_index++;
      check_next_index = check_next_index % PDCP_RECV_WINDOW_MAX_LEN;
      cur_sdu = entity->pdcp_recv_window[check_next_index];
    }
    entity->rx_deliv = entity->rx_deliv + real_deliver_num; 
    entity->window_start_index = (entity->window_start_index + real_deliver_num) % PDCP_RECV_WINDOW_MAX_LEN;
    entity->pdcp_recv_in_window_count -= real_deliver_num;    

    LOG_D(PDCP, "step2: rx_deliv %d, window_start_index %d, real_deliver_num %d, pdcp_recv_in_window_count %d\n", 
          entity->rx_deliv,entity->window_start_index,real_deliver_num,entity->pdcp_recv_in_window_count);
  }

  int rcvd_count_in_window = (entity->window_start_index + rcvd_count - entity->rx_deliv) % PDCP_RECV_WINDOW_MAX_LEN;

  if (entity->pdcp_recv_window[rcvd_count_in_window] != NULL)
  {
    LOG_W(PDCP, "discard NR PDU rcvd_count=%d, already recv! \n", rcvd_count);
    return;
  }

  LOG_D(PDCP, "recv NR PDU rcvd_count=%d, rx_deliv %d, window_start_index %d, rcvd_count_in_window %d, pdcp_recv_in_window_count %d\n", 
  rcvd_count,entity->rx_deliv,entity->window_start_index,rcvd_count_in_window,entity->pdcp_recv_in_window_count);

  cur_sdu = nr_pdcp_new_sdu(rcvd_count,
                        (char *)buffer + header_size,
                        size - header_size - integrity_size);

  entity->pdcp_recv_window[rcvd_count_in_window] = cur_sdu;
  entity->pdcp_recv_in_window_count ++;
  if (rcvd_count == entity->rx_deliv) 
  {
    /* deliver all SDUs starting from rx_deliv up to discontinuity or end of list */
    int delivery_index=rcvd_count_in_window;
    int delivery_count=0;

    while (cur_sdu != NULL)
    {
      entity->deliver_sdu(entity->deliver_sdu_data, entity,
                          cur_sdu->buffer, cur_sdu->size);
      nr_pdcp_free_sdu(cur_sdu);
      entity->pdcp_recv_window[delivery_index]= NULL;
      delivery_count++;
      delivery_index++;
      delivery_index = delivery_index % PDCP_RECV_WINDOW_MAX_LEN;
      cur_sdu = entity->pdcp_recv_window[delivery_index];

    }

    entity->rx_deliv = rcvd_count+delivery_count;
    entity->window_start_index = delivery_index;
    entity->pdcp_recv_in_window_count -= delivery_count;
  }

  if (entity->t_reordering_start != 0 && entity->rx_deliv >= entity->rx_reord) {
    /* stop and reset t-Reordering */
    entity->t_reordering_start = 0;
  }

  if (entity->t_reordering_start == 0 && entity->rx_deliv < entity->rx_next) {
    entity->rx_reord = entity->rx_next;
    entity->t_reordering_start = entity->t_current;
  }
}

static void nr_pdcp_entity_recv_sdu(nr_pdcp_entity_t *entity,
                                    char *buffer, int size, int sdu_id)
{
  uint32_t count;
  int      sn;
  int      header_size;
  int      integrity_size;
  char     buf[size + 3 + 4];
  int      dc_bit;

  count = entity->tx_next;
  sn = entity->tx_next & entity->sn_max;

  /* D/C bit is only to be set for DRBs */
  if (entity->type == NR_PDCP_DRB_AM || entity->type == NR_PDCP_DRB_UM) {
    dc_bit = 0x80;
  } else {
    dc_bit = 0;
  }

  if (entity->sn_size == 12) {
    buf[0] = dc_bit | ((sn >> 8) & 0xf);
    buf[1] = sn & 0xff;
    header_size = 2;
  } else {
    buf[0] = dc_bit | ((sn >> 16) & 0x3);
    buf[1] = (sn >> 8) & 0xff;
    buf[2] = sn & 0xff;
    header_size = 3;
  }

  /* SRBs always have MAC-I, even if integrity is not active */
  if (entity->has_integrity || entity->type == NR_PDCP_SRB) {
    integrity_size = 4;
  } else {
    integrity_size = 0;
  }

  memcpy(buf + header_size, buffer, size);

  if (entity->has_integrity)
    entity->integrity(entity->integrity_context,
                      (unsigned char *)buf + header_size + size,
                      (unsigned char *)buf, header_size + size,
                      entity->rb_id, count, entity->is_gnb ? 1 : 0);

  // set MAC-I to 0 for SRBs with integrity not active
  else if (integrity_size == 4)
    memset(buf + header_size + size, 0, 4);

  if (entity->has_ciphering)
    entity->cipher(entity->security_context,
                   (unsigned char *)buf + header_size, size + integrity_size,
                   entity->rb_id, count, entity->is_gnb ? 1 : 0);

  entity->tx_next++;

  entity->deliver_pdu(entity->deliver_pdu_data, entity, buf,
                      header_size + size + integrity_size, sdu_id);
}

/* may be called several times, take care to clean previous settings */
static void nr_pdcp_entity_set_security(nr_pdcp_entity_t *entity,
                                        int integrity_algorithm,
                                        char *integrity_key,
                                        int ciphering_algorithm,
                                        char *ciphering_key)
{
  if (integrity_algorithm != -1)
    entity->integrity_algorithm = integrity_algorithm;
  if (ciphering_algorithm != -1)
    entity->ciphering_algorithm = ciphering_algorithm;
  if (integrity_key != NULL)
    memcpy(entity->integrity_key, integrity_key, 16);
  if (ciphering_key != NULL)
    memcpy(entity->ciphering_key, ciphering_key, 16);

  if (integrity_algorithm == 0) {
    entity->has_integrity = 0;
    if (entity->free_integrity != NULL)
      entity->free_integrity(entity->integrity_context);
    entity->free_integrity = NULL;
  }

  if (integrity_algorithm != 0 && integrity_algorithm != -1) {
    if (integrity_algorithm != 2) {
      LOG_E(PDCP, "FATAL: only nia2 supported for the moment\n");
      exit(1);
    }
    entity->has_integrity = 1;
    if (entity->free_integrity != NULL)
      entity->free_integrity(entity->integrity_context);
    entity->integrity_context = nr_pdcp_integrity_nia2_init(entity->integrity_key);
    entity->integrity = nr_pdcp_integrity_nia2_integrity;
    entity->free_integrity = nr_pdcp_integrity_nia2_free_integrity;
  }

  if (ciphering_algorithm == 0) {
    entity->has_ciphering = 0;
    if (entity->free_security != NULL)
      entity->free_security(entity->security_context);
    entity->free_security = NULL;
  }

  if (ciphering_algorithm != 0 && ciphering_algorithm != -1) {
    if (ciphering_algorithm != 2) {
      LOG_E(PDCP, "FATAL: only nea2 supported for the moment\n");
      exit(1);
    }
    entity->has_ciphering = 1;
    if (entity->free_security != NULL)
      entity->free_security(entity->security_context);
    entity->security_context = nr_pdcp_security_nea2_init(entity->ciphering_key);
    entity->cipher = nr_pdcp_security_nea2_cipher;
    entity->free_security = nr_pdcp_security_nea2_free_security;
  }
}

static void check_t_reordering(nr_pdcp_entity_t *entity)
{
  uint32_t count;

  /* if t_reordering is set to "infinity" (seen as -1) then do nothing */
  if (entity->t_reordering == -1)
    return;

  if (entity->t_reordering_start == 0
      || entity->t_current <= entity->t_reordering_start + entity->t_reordering)
    return;

  /* stop timer */
  entity->t_reordering_start = 0;

  /* deliver all SDUs with count < rx_reord */
  while (entity->rx_list != NULL && entity->rx_list->count < entity->rx_reord) {
    nr_pdcp_sdu_t *cur = entity->rx_list;
    entity->deliver_sdu(entity->deliver_sdu_data, entity,
                        cur->buffer, cur->size);
    entity->rx_list = cur->next;
    entity->rx_size -= cur->size;
    nr_pdcp_free_sdu(cur);
  }

  /* deliver all SDUs starting from rx_reord up to discontinuity or end of list */
  count = entity->rx_reord;
  while (entity->rx_list != NULL && count == entity->rx_list->count) {
    nr_pdcp_sdu_t *cur = entity->rx_list;
    entity->deliver_sdu(entity->deliver_sdu_data, entity,
                        cur->buffer, cur->size);
    entity->rx_list = cur->next;
    entity->rx_size -= cur->size;
    nr_pdcp_free_sdu(cur);
    count++;
  }

  entity->rx_deliv = count;

  if (entity->rx_deliv < entity->rx_next) {
    entity->rx_reord = entity->rx_next;
    entity->t_reordering_start = entity->t_current;
  }
}

void nr_pdcp_entity_set_time(struct nr_pdcp_entity_t *entity, uint64_t now)
{
  entity->t_current = now;

  check_t_reordering(entity);
}

void nr_pdcp_entity_delete(nr_pdcp_entity_t *entity)
{
  nr_pdcp_sdu_t *cur = entity->rx_list;
  while (cur != NULL) {
    nr_pdcp_sdu_t *next = cur->next;
    nr_pdcp_free_sdu(cur);
    cur = next;
  }
  if (entity->free_security != NULL)
    entity->free_security(entity->security_context);
  if (entity->free_integrity != NULL)
    entity->free_integrity(entity->integrity_context);
  free(entity);
}

nr_pdcp_entity_t *new_nr_pdcp_entity(
    nr_pdcp_entity_type_t type,
    int is_gnb, int rb_id, int pdusession_id,int has_sdap,
    int has_sdapULheader, int has_sdapDLheader,
    void (*deliver_sdu)(void *deliver_sdu_data, struct nr_pdcp_entity_t *entity,
                        char *buf, int size),
    void *deliver_sdu_data,
    void (*deliver_pdu)(void *deliver_pdu_data, struct nr_pdcp_entity_t *entity,
                        char *buf, int size, int sdu_id),
    void *deliver_pdu_data,
    int sn_size,
    int t_reordering,
    int discard_timer,
    int ciphering_algorithm,
    int integrity_algorithm,
    unsigned char *ciphering_key,
    unsigned char *integrity_key)
{
  nr_pdcp_entity_t *ret;

  ret = calloc(1, sizeof(nr_pdcp_entity_t));
  if (ret == NULL) {
    LOG_E(PDCP, "%s:%d:%s: out of memory\n", __FILE__, __LINE__, __FUNCTION__);
    exit(1);
  }

  ret->type = type;

  ret->recv_pdu     = nr_pdcp_entity_recv_pdu;
  ret->recv_sdu     = nr_pdcp_entity_recv_sdu;
  ret->set_security = nr_pdcp_entity_set_security;
  ret->set_time     = nr_pdcp_entity_set_time;

  ret->delete_entity = nr_pdcp_entity_delete;
  
  ret->deliver_sdu = deliver_sdu;
  ret->deliver_sdu_data = deliver_sdu_data;

  ret->deliver_pdu = deliver_pdu;
  ret->deliver_pdu_data = deliver_pdu_data;

  ret->rb_id         = rb_id;
  ret->pdusession_id = pdusession_id;
  ret->has_sdap      = has_sdap;
  ret->has_sdapULheader = has_sdapULheader;
  ret->has_sdapDLheader = has_sdapDLheader;
  ret->sn_size       = sn_size;
  ret->t_reordering  = t_reordering;
  ret->discard_timer = discard_timer;

  ret->sn_max        = (1 << sn_size) - 1;
  ret->window_size   = 1 << (sn_size - 1);

  ret->is_gnb = is_gnb;

  ret->pdcp_recv_in_window_count = 0;
  ret->window_start_index = 0;
  memset(ret->pdcp_recv_window, 0, sizeof(ret->pdcp_recv_window));
//  printf("sizeof(ret->pdcp_recv_window) %d \n",sizeof(ret->pdcp_recv_window));

  nr_pdcp_entity_set_security(ret,
                              integrity_algorithm, (char *)integrity_key,
                              ciphering_algorithm, (char *)ciphering_key);

  return ret;
}
