/****************************************************************************
 *
 * Copyright 2020 Charmed Labs.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
//
// This file is for defining the Block struct and the Pixy template class version 2.
// (TPixy2).  TPixy takes a communication link as a template parameter so that
// all communication modes (SPI, I2C and UART) can share the same code.
//

#include "Modules/mDelay.h"
#include "Utils/util.h"

#ifndef _PIXY2VIDEO_H
#define _PIXY2VIDEO_H

#define VIDEO_REQUEST_GET_RGB 0x70
#define VIDEO_FRAME_HEIGHT 208
#define VIDEO_FRAME_WIDTH 316

template <class LinkType>
class TPixy2;

template <class LinkType>
class Pixy2Video
{
public:
  Pixy2Video(TPixy2<LinkType> *pixy)
  {
    m_pixy = pixy;
  }

  int8_t getRGB(uint8_t *frame, bool saturate = true);

private:
  TPixy2<LinkType> *m_pixy;
};

template <class LinkType>
int8_t Pixy2Video<LinkType>::getRGB(uint8_t *frame, bool saturate)
{
  while (1)
  {
    // char test[64];
    // sprintf(test, "%p, %d\n\r", m_pixy->m_bufPayload, x);
    // print_string(test);

    uint8_t r;
    uint8_t g;
    uint8_t b;

    *(int16_t *)(m_pixy->m_bufPayload + 0) = x;
    *(int16_t *)(m_pixy->m_bufPayload + 2) = y;
    *(m_pixy->m_bufPayload + 4) = saturate;
    m_pixy->m_length = 5;
    m_pixy->m_type = VIDEO_REQUEST_GET_RGB;
    m_pixy->sendPacket();

    for (int y_test = 0; y_test < 208; y_test++){
      for(int x_test = 0; x_test < 316; x_test++){
        if (m_pixy->recvPacket() == 0)
        {
          if (m_pixy->m_type == PIXY_TYPE_RESPONSE_RESULT && m_pixy->m_length == 4)
          {
            *b = *(m_pixy->m_buf + 0);
            *g = *(m_pixy->m_buf + 1);
            *r = *(m_pixy->m_buf + 2);

            int gray_scale = (r + g + b) / 3;
            frame[(y_test * VIDEO_FRAME_WIDTH) + x_test] = gray_scale;
          }
          // deal with program changing
          else if (m_pixy->m_type == PIXY_TYPE_RESPONSE_ERROR && (int8_t)m_pixy->m_buf[0] == PIXY_RESULT_PROG_CHANGING)
          {
            print_string("Error\n\r");
            usleep(500); // don't be a drag
            continue;
          }
        }
      }
    }

    return 0;
  }
}

#endif
