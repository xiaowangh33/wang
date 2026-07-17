#include "wd_protocol.h"

#include <string.h>

#define WD_STATIC_ASSERT(name, condition) typedef char wd_static_assert_##name[(condition) ? 1 : -1]

WD_STATIC_ASSERT(packet_header_size, sizeof(WdPacketHeader) == 16u);
WD_STATIC_ASSERT(joint_command_size, sizeof(WdJointCommand) == 20u);
WD_STATIC_ASSERT(setpoint_payload_size, sizeof(WdSetpointPayload) == 352u);
WD_STATIC_ASSERT(joint_feedback_size, sizeof(WdJointFeedback) == 24u);
WD_STATIC_ASSERT(feedback_payload_size, sizeof(WdFeedbackPayload) == 1172u);

static uint32_t wd_read_u32_le(const uint8_t *p) {
  return ((uint32_t)p[0]) |
         ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

static uint16_t wd_read_u16_le(const uint8_t *p) {
  return (uint16_t)(((uint16_t)p[0]) | ((uint16_t)p[1] << 8));
}

static void wd_write_u16_le(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

void wd_protocol_parser_init(WdProtocolParser *parser) {
  if (parser == NULL) {
    return;
  }
  memset(parser, 0, sizeof(*parser));
}

int wd_protocol_parser_has_pending(const WdProtocolParser *parser) {
  return (parser != NULL && parser->size > 0u) ? 1 : 0;
}

int wd_protocol_buffer_starts_like_packet(const uint8_t *data, uint32_t len) {
  if (data == NULL || len < 4u) {
    return 0;
  }
  return (wd_read_u32_le(data) == WD_PROTOCOL_MAGIC) ? 1 : 0;
}

uint16_t wd_protocol_crc16_ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFFu;
  size_t i;

  if (data == NULL) {
    return crc;
  }
  for (i = 0u; i < len; ++i) {
    uint8_t bit;
    crc ^= (uint16_t)data[i] << 8;
    for (bit = 0u; bit < 8u; ++bit) {
      if ((crc & 0x8000u) != 0u) {
        crc = (uint16_t)((crc << 1) ^ 0x1021u);
      } else {
        crc = (uint16_t)(crc << 1);
      }
    }
  }
  return crc;
}

int wd_protocol_build_packet(uint8_t type,
                             uint32_t seq,
                             const void *payload,
                             uint16_t payload_size,
                             uint8_t *out,
                             uint16_t out_capacity,
                             uint16_t *out_size) {
  WdPacketHeader header;
  uint16_t crc;
  uint16_t total;

  if (out == NULL || out_size == NULL) {
    return -1;
  }
  if (payload_size > WD_PROTOCOL_MAX_PAYLOAD) {
    return -2;
  }
  if (payload_size > 0u && payload == NULL) {
    return -3;
  }
  total = (uint16_t)(sizeof(WdPacketHeader) + payload_size);
  if (out_capacity < total) {
    return -4;
  }

  memset(&header, 0, sizeof(header));
  header.magic = WD_PROTOCOL_MAGIC;
  header.version = WD_PROTOCOL_VERSION;
  header.type = type;
  header.payload_size = payload_size;
  header.seq = seq;
  header.crc16 = 0u;
  header.reserved = 0u;

  memcpy(out, &header, sizeof(header));
  if (payload_size > 0u) {
    memcpy(out + sizeof(header), payload, payload_size);
  }
  crc = wd_protocol_crc16_ccitt(out, total);
  wd_write_u16_le(out + 12u, crc);
  *out_size = total;
  return 0;
}

static void wd_parser_drop_bytes(WdProtocolParser *parser, uint16_t count) {
  if (count >= parser->size) {
    parser->size = 0u;
    return;
  }
  memmove(parser->buffer, parser->buffer + count, parser->size - count);
  parser->size = (uint16_t)(parser->size - count);
}

static int wd_parser_find_magic(const WdProtocolParser *parser) {
  uint16_t i;
  if (parser->size < 4u) {
    return 0;
  }
  for (i = 0u; i <= (uint16_t)(parser->size - 4u); ++i) {
    if (wd_read_u32_le(parser->buffer + i) == WD_PROTOCOL_MAGIC) {
      return (int)i;
    }
  }
  return -1;
}

static int wd_protocol_process_buffer(WdProtocolParser *parser,
                                      WdProtocolPacketCallback callback,
                                      void *context) {
  int decoded = 0;

  while (parser->size >= sizeof(WdPacketHeader)) {
    uint16_t payload_size;
    uint16_t total_size;
    uint16_t received_crc;
    uint16_t computed_crc;
    uint8_t saved_crc[2];
    int magic_offset;
    WdPacketHeader header;

    magic_offset = wd_parser_find_magic(parser);
    if (magic_offset < 0) {
      wd_parser_drop_bytes(parser, (uint16_t)(parser->size - 3u));
      return decoded;
    }
    if (magic_offset > 0) {
      wd_parser_drop_bytes(parser, (uint16_t)magic_offset);
    }
    if (parser->size < sizeof(WdPacketHeader)) {
      return decoded;
    }

    memcpy(&header, parser->buffer, sizeof(header));
    if (header.version != WD_PROTOCOL_VERSION) {
      ++parser->bad_packets;
      wd_parser_drop_bytes(parser, 1u);
      continue;
    }

    payload_size = wd_read_u16_le(parser->buffer + 6u);
    if (payload_size > WD_PROTOCOL_MAX_PAYLOAD) {
      ++parser->bad_packets;
      wd_parser_drop_bytes(parser, 1u);
      continue;
    }

    total_size = (uint16_t)(sizeof(WdPacketHeader) + payload_size);
    if (parser->size < total_size) {
      return decoded;
    }

    received_crc = wd_read_u16_le(parser->buffer + 12u);
    saved_crc[0] = parser->buffer[12];
    saved_crc[1] = parser->buffer[13];
    parser->buffer[12] = 0u;
    parser->buffer[13] = 0u;
    computed_crc = wd_protocol_crc16_ccitt(parser->buffer, total_size);
    parser->buffer[12] = saved_crc[0];
    parser->buffer[13] = saved_crc[1];

    if (computed_crc != received_crc) {
      ++parser->crc_errors;
      wd_parser_drop_bytes(parser, 1u);
      continue;
    }

    ++parser->packets;
    ++decoded;
    if (callback != NULL) {
      callback(header.type,
               wd_read_u32_le(parser->buffer + 8u),
               parser->buffer + sizeof(WdPacketHeader),
               payload_size,
               context);
    }
    wd_parser_drop_bytes(parser, total_size);
  }

  return decoded;
}

int wd_protocol_feed(WdProtocolParser *parser,
                     const uint8_t *data,
                     uint32_t len,
                     WdProtocolPacketCallback callback,
                     void *context) {
  uint32_t copied = 0u;
  int decoded = 0;

  if (parser == NULL || data == NULL || len == 0u) {
    return 0;
  }

  while (copied < len) {
    uint32_t space = WD_PROTOCOL_MAX_PACKET - parser->size;
    uint32_t chunk = len - copied;
    if (space == 0u) {
      ++parser->bad_packets;
      parser->size = 0u;
      space = WD_PROTOCOL_MAX_PACKET;
    }
    if (chunk > space) {
      chunk = space;
    }
    memcpy(parser->buffer + parser->size, data + copied, chunk);
    parser->size = (uint16_t)(parser->size + chunk);
    copied += chunk;
    decoded += wd_protocol_process_buffer(parser, callback, context);
  }

  return decoded;
}
