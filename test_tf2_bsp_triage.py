import lzma
import struct
import unittest

from tf2_bsp_triage import BspParseError, _decompress_source_lzma


FILTER = {"id": lzma.FILTER_LZMA1, "dict_size": 1 << 20, "lc": 3, "lp": 0, "pb": 2}


def source_lzma(payload: bytes, *, actual_size=None, compressed_size=None) -> bytes:
    compressed = lzma.compress(payload, format=lzma.FORMAT_RAW, filters=[FILTER])
    properties = bytes([FILTER["lc"] + 9 * (FILTER["lp"] + 5 * FILTER["pb"])])
    properties += struct.pack("<I", FILTER["dict_size"])
    return struct.pack(
        "<4sII", b"LZMA", len(payload) if actual_size is None else actual_size,
        len(compressed) if compressed_size is None else compressed_size,
    ) + properties + compressed


class SourceLzmaTests(unittest.TestCase):
    def test_decompresses_complete_lump(self):
        payload = b"plane record" * 20

        self.assertEqual(_decompress_source_lzma(source_lzma(payload)), payload)

    def test_rejects_missing_declared_compressed_payload(self):
        lump = source_lzma(b"geometry", compressed_size=1000)

        with self.assertRaisesRegex(BspParseError, "only .* are present"):
            _decompress_source_lzma(lump)

    def test_rejects_short_decompressed_output(self):
        lump = source_lzma(b"geometry", actual_size=100)

        with self.assertRaisesRegex(BspParseError, "decompression produced 8 bytes"):
            _decompress_source_lzma(lump)


if __name__ == "__main__":
    unittest.main()
