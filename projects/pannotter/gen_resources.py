#!/usr/bin/env python3
import string

c_font_tpl = """\
static const font_glyph_t font_%(name)s_glyphs[] PROGMEM = {
  %(glyphs)s
};

static const uint8_t font_%(name)s_data[] PROGMEM = {
  %(data)s
};

const font_t font_%(name)s = {
  .height = %(height)s,
  .glyphs = font_%(name)s_glyphs,
  .data = font_%(name)s_data,
};
"""

c_image_tpl = """\
static const pixel_t image_%(name)s_data[] PROGMEM = {
  %(data)s
};

const image_t image_%(name)s = {
  .width = %(width)s,
  .height = %(height)s,
  .data = image_%(name)s_data,
};
"""


class Font:

    all_chars = sorted(string.ascii_letters + string.digits + string.punctuation + " ", key=ord)

    def __init__(self, name, chars, bitmap):
        self.name = name

        # parse bitamp
        rows = [line.split("|") for line in bitmap.split("\n")]
        # split on separator rows
        sep = [i for i, row in enumerate(rows) if len(row) == 1 and not row[0].replace("-", "")]

        max_height = 0
        chars_data = []  # list of char rows
        chars_width = []
        prev_i = 0
        for i in [0] + sep + [len(sep)]:
            group = rows[prev_i:i]
            if not group:
                continue
            assert all(len(l) == len(group[0]) for l in group), "all rows of a group must have the same vertical separator count"
            for char_rows in zip(*group):
                width = len(char_rows[0])
                assert all(len(l) == width for l in char_rows), "vertical separators must be aligned"
                if width:
                    chars_width.append(width)
                    chars_data.append("".join(char_rows))

            max_height = max(max_height, i - prev_i)
            prev_i = i + 1

        # set instance fields
        self.height = max_height
        self.glyphs = {}  # {char: (width, data_rows)}
        char_to_value = {" ": 0, "#": 0xff}

        chars = chars.replace("\n", "")
        assert len(chars_data) == len(chars), "wrong number of chars in bitmap"
        for char, data, width in zip(chars, chars_data, chars_width):
            assert char not in self.glyphs, "duplicate character"
            data = data.ljust(width * self.height, " ")
            data_rows = [bytes(char_to_value[c] for c in data[i:i+width]) for i in range(0, len(data), width)]
            self.glyphs[char] = (width, data_rows)

    def gen_c(self, direction):
        def pixel_bytes(rows):
            if direction in 'hr':
                return bytes(b for row in rows for b in row)
            elif direction == 'v':
                return bytes(b for col in zip(*rows) for b in col)
            else:
                raise ValueError("invalid direction")

        data_bytes = []
        glyphs = []
        offset = 0
        for c in self.all_chars:
            glyph = self.glyphs.get(c)
            if glyph:
                width, data_rows = glyph
                glyphs.append("  /* %c */ {%d, %d}," % (c, width, offset))
                data_bytes.extend(pixel_bytes(data_rows))
                offset += width * self.height
            else:
                glyphs.append("  /* %c */ {0, 0}," % c)

        data = ["0x%02x," % b for b in data_bytes]
        # split data in groups of 12 bytes
        data = [" ".join(data[i:i+12]) for i in range(0, len(data), 12)]

        return c_font_tpl % dict(
            name = self.name,
            height = self.height,
            glyphs = "\n  ".join(glyphs),
            data = "\n  ".join(data),
        )


class Image:
    def __init__(self, name, palette, rgb):
        self.name = name

        # check palette
        for c, color in palette.items():
            assert c != ' ' and len(c) == 1, "wrong palette index"
            assert len(color) == 3, "wrong color value"
            assert all(0 <= v <= 255 for v in color), "wrong color component value"

        # parse rows, get size
        rows = rgb.strip("\n").split("\n")
        self.height = len(rows)
        assert self.height, "empty image"
        self.width = max(len(row) for row in rows)
        rows = [r.ljust(self.width, ' ') for r in rows]

        # joint all pixels, check for invalid ones
        for c in set(p for row in pixels for p in row):
            assert c == ' ' or c in palette, "unknown color character: %r" % c
        self.pixels = [[palette.get(c, (0, 0, 0)) for c in row] for row in pixels]

    def gen_c(self):
        if direction in 'hr':
            pixel_iter = (p for row in self.pixels for p in row)
        elif direction == 'v':
            pixel_iter = (p for col in zip(*self.pixels) for p in col)
        else:
            raise ValueError("invalid direction")

        data = ["{0x%02x,0x%02x,0x%02x}," % b for b in pixel_iter]
        # split data in groups of 4 pixels
        data = [" ".join(data[i:i+12]) for i in range(0, len(data), 12)]

        return c_image_tpl % dict(
            name = self.name,
            height = self.height,
            width = self.width,
            data = "\n  ".join(data),
        )


font_score_chars = "0123456789 ?-"
font_score_bitmap = """\
###| # |###|###|# #|###|###|###|###|###|   |###|   |
# #|## |  #|  #|# #|#  |#  |  #|# #|# #|   |# #|   |
# #| # |  #|  #|# #|#  |#  |  #|# #|# #|   |  #|   |
# #| # |  #|  #|# #|#  |#  |  #|# #|# #|   |  #|   |
# #| # |###|###|###|###|###| # |###|###|   | # | # |
# #| # |#  |  #|  #|  #|# #| # |# #|  #|   | # |   |
# #| # |#  |  #|  #|  #|# #| # |# #|  #|   | # |   |
# #| # |#  |  #|  #|  #|# #| # |# #|  #|   |   |   |
###|###|###|###|  #|###|###| # |###|###|   | # |   |
"""


def main():
    pixel_direction = 'r'  # h, v, r

    fonts = [
        Font("score", font_score_chars, font_score_bitmap),
    ]
    images = []

    for font in fonts:
        print("")
        print(font.gen_c(pixel_direction))

    for image in images:
        print("")
        print(image.gen_c(pixel_direction))


if __name__ == "__main__":
    main()

