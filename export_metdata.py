#!/usr/bin/env python3
"""
Đọc file MCAP và xuất metadata_sample.yaml:
- Liệt kê từng topic: tên, schema, số message (count)
- Lấy giá trị mẫu cụ thể từ message ĐẦU TIÊN của mỗi topic

Cài: pip install mcap pyyaml
Dùng:
  python mcap_to_metadata.py video.mcap
  python mcap_to_metadata.py video.mcap metadata_sample.yaml
"""
import sys
import json
import base64
import argparse
from collections import OrderedDict

import yaml
from mcap.reader import make_reader


def truncate_bytes(obj, max_len=64):
    """Rút gọn các trường bytes/data dài để YAML đọc được."""
    if isinstance(obj, dict):
        return {k: truncate_bytes(v, max_len) for k, v in obj.items()}
    if isinstance(obj, list):
        if len(obj) > 16:
            return obj[:16] + [f"... ({len(obj)} phần tử)"]
        return [truncate_bytes(v, max_len) for v in obj]
    if isinstance(obj, (bytes, bytearray)):
        return f"<{len(obj)} bytes> {base64.b64encode(obj[:16]).decode()}..."
    return obj


def decode_message(schema, message):
    """Giải mã message theo encoding của schema. Trả về dict hoặc raw."""
    enc = (schema.encoding if schema else "") or ""
    data = message.data

    # JSON encoding
    if "json" in enc.lower():
        try:
            return json.loads(data.decode("utf-8"))
        except Exception:
            return {"_raw_len": len(data)}

    # protobuf -> không decode sâu (cần schema .proto), chỉ tóm tắt
    if "protobuf" in enc.lower():
        return {"_protobuf_bytes": len(data)}

    # khác
    try:
        return json.loads(data.decode("utf-8"))
    except Exception:
        return {"_raw_len": len(data)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mcap_file")
    ap.add_argument("output", nargs="?", default="metadata_sample.yaml")
    args = ap.parse_args()

    counts = OrderedDict()       # topic -> count
    schema_of = {}               # topic -> (schema_name, encoding)
    first_msg = {}               # topic -> decoded sample (message đầu tiên)

    with open(args.mcap_file, "rb") as f:
        reader = make_reader(f)

        for schema, channel, message in reader.iter_messages():
            topic = channel.topic
            counts[topic] = counts.get(topic, 0) + 1
            if topic not in schema_of:
                schema_of[topic] = (
                    schema.name if schema else "(none)",
                    (schema.encoding if schema else "") or channel.message_encoding,
                )
            if topic not in first_msg:
                first_msg[topic] = decode_message(schema, message)

    # ---- Xây dựng cấu trúc YAML ----
    topics_out = []
    for topic in sorted(counts.keys()):
        sname, senc = schema_of[topic]
        sample = truncate_bytes(first_msg.get(topic, {}))
        topics_out.append(OrderedDict([
            ("name", topic),
            ("type", sname),
            ("encoding", senc),
            ("count", counts[topic]),
            ("sample_value", sample),
        ]))

    out = OrderedDict([("topics", topics_out)])

    # YAML giữ thứ tự key (OrderedDict)
    class OrderedDumper(yaml.SafeDumper):
        pass

    def _dict_repr(dumper, data):
        return dumper.represent_mapping(
            yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, data.items())
    OrderedDumper.add_representer(OrderedDict, _dict_repr)

    with open(args.output, "w", encoding="utf-8") as f:
        yaml.dump(out, f, Dumper=OrderedDumper,
                  allow_unicode=True, sort_keys=False, default_flow_style=False)

    print(f"Đã xuất {args.output}")
    print(f"Tổng số topic: {len(counts)}")
    for topic in sorted(counts.keys()):
        print(f"  {topic:35s} {schema_of[topic][0]:30s} count={counts[topic]}")


if __name__ == "__main__":
    main()