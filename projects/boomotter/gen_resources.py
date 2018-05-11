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
        chars_data = []
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
        self.data = b""
        self.glyphs = {}  # {char: (width, offset)}
        char_to_value = {" ": 0, "#": 0xff}

        chars = chars.replace("\n", "")
        assert len(chars_data) == len(chars), "wrong number of chars in bitmap"
        for char, data, width in zip(chars, chars_data, chars_width):
            assert char not in self.glyphs, "duplicate character"
            data = data.ljust(width * self.height, " ")
            self.glyphs[char] = (width, len(self.data))
            self.data += bytes(char_to_value[c] for c in data)

    def gen_c(self):
        glyphs = []
        for c in self.all_chars:
            glyph = self.glyphs.get(c)
            if glyph:
                glyphs.append("  /* %c */ {%d, %d}," % (c, glyph[0], glyph[1]))
            else:
                glyphs.append("  /* %c */ {0, 0}," % c)

        data = ["0x%02x," % b for b in self.data]
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
        pixels = ''.join(rows)
        for c in set(pixels):
            assert c == ' ' or c in palette, "unknown color character: %r" % c
        self.pixels = [palette.get(c, (0, 0, 0)) for c in pixels]

    def gen_c(self):
        data = ["{0x%02x,0x%02x,0x%02x}," % b for b in self.pixels]
        # split data in groups of 4 pixels
        data = [" ".join(data[i:i+12]) for i in range(0, len(data), 12)]

        return c_image_tpl % dict(
            name = self.name,
            height = self.height,
            width = self.width,
            data = "\n  ".join(data),
        )



font_base_orig_chars = """\
ABCDEFGHIJKLMNOPQRSTUVWXYZ
 0123456789
"""

font_base_orig_bitmap = """\
 ## |### | ##|### |###|###| ###|#  #|###|  ##|#  #|#  |#   #|#  #| ## |### | ## |### | ###|###|#  #|#  #|#   #|#  #|#  #|###|
#  #|#  #|#  |#  #|#  |#  |#   |#  #| # |   #|# # |#  |## ##|## #|#  #|#  #|#  #|#  #|#   | # |#  #|#  #|# # #|#  #|#  #|  #|
#  #|### |#  |#  #|###|###|# ##|####| # |   #|##  |#  |# # #|# ##|#  #|#  #|#  #|#  #| ## | # |#  #|# # |# # #| ## | ###| # |
####|#  #|#  |#  #|#  |#  |#  #|#  #| # |#  #|# # |#  |#   #|#  #|#  #|### |# ##|### |   #| # |#  #|# # |# # #|#  #|   #|#  |
#  #|### | ##|### |###|#  | ###|#  #|###| ## |#  #|###|#   #|#  #| ## |#   | ###|#  #|### | # | ## | #  | # # |#  #| ## |###|
    |    |   |    |   |   |    |    |   |    |    |   |     |    |    |    |    |    |    |   |    |    |     |    |    |   |
-----------------------------------------------------------------------------------------------------------------------------
   | ## |##|### |### |  # |####| ## |####| ## | ## |
   |#  #| #|   #|   #| ## |#   |#   |   #|#  #|#  #|
   |#  #| #| ## | ## |# # |### |### |  # | ## | ###|
   |#  #| #|#   |   #|####|   #|#  #| #  |#  #|   #|
   | ## | #|####|### |  # |### | ## | #  | ## | ## |
   |    |  |    |    |    |    |    |    |    |    |
----------------------------------------------------
"""

font_base_chars = """\
ABCDEFGHIJKLMNOPQRSTUVWXYZ
0123456789
 -+*/=%"'#@&_
()[]{}<>
,.;:?!\\|
"""

font_base_bitmap = """\
 ## |### | ##|### |###|###| ###|#  #|###|  ##|#  #|#  |#   #|#  #| ## |### | ## |### | ###|###|#  #|#  #|#   #|#  #|#  #|###|
#  #|#  #|#  |#  #|#  |#  |#   |#  #| # |   #|# # |#  |## ##|## #|#  #|#  #|#  #|#  #|#   | # |#  #|#  #|# # #|#  #|#  #|  #|
####|### |#  |#  #|###|###|# ##|####| # |   #|##  |#  |# # #|# ##|#  #|### |#  #|### | ## | # |#  #|# # |# # #| ## | ## | # |
#  #|#  #|#  |#  #|#  |#  |#  #|#  #| # |#  #|# # |#  |#   #|#  #|#  #|#   |# ##|# # |   #| # |#  #|# # |# # #|#  #| #  |#  |
#  #|### | ##|### |###|#  | ###|#  #|###| ## |#  #|###|#   #|#  #| ## |#   | ###|#  #|### | # | ## | #  | # # |#  #|#   |###|
-----------------------------------------------------------------------------------------------------------------------------
 ## | # |### |### |  # |####| ## |####| ## | ## |
#  #|## |   #|   #| ## |#   |#   |   #|#  #|#  #|
#  #| # | ## | ## |# # |### |### |  # | ## | ###|
#  #| # |#   |   #|####|   #|#  #| #  |#  #|   #|
 ## |###|####|### |  # |### | ## | #  | ## | ## |
----------------------------------------------------------
   |  |   |   |    #|   |# #|# #|#| # # | ### | ##  |    |
   |  | # |# #|   # |###|  #|# #|#|#####|#   #|#    |    |
   |##|###| # |  #  |   | # |   | | # # |# ###| ## #|    |
   |  | # |# #| #   |###|#  |   | |#####|# # #|#  # |    |
   |  |   |   |#    |   |# #|   | | # # | ### | ## #|####|
----------------------------------------------------------
  #|#  | ##|## | ##|## |  #|#  |
 # | # | # | # | # | # | # | # |
 # | # | # | # |#  |  #|#  |  #|
 # | # | # | # | # | # | # | # |
  #|#  | ##|## | ##|## |  #|#  |
--------------------------------
   |  |  |  |### |# |#    | # |
   |  |# |# |   #|# | #   | # |
   |  |  |  | ## |# |  #  | # |
 # |# |# |# |    |  |   # | # |
#  |  |# |  | #  |# |    #| # |
-------------------------------
"""

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


image_bug_palette = {
    '#': (25,25,25),
    'x': (10,10,10),
    'o': (50,0,0),
    '|': (10,10,10),
    '.': (1,0,0),
}
image_bug1_rgb = """\
    ###
x  #...#  x
 x#.o.o.#x
  #.....#
  #..|..#
 x#..|..#x
x  #.|.#  x
    #|#
"""
image_bug2_rgb = """\
    ###
   #...#
xx#.o.o.#xx
  #.....#
  #..|..#
xx#..|..#xx
   #.|.#
    #|#
"""

image_nyancat_palette = {
    'x': (0x20,0x20,0x20),
    '*': (0x20,0x01,0x01),
    'o': (0x03,0x00,0x03),
    '.': (0x10,0x03,0x03),
}
image_nyancat_rgb1 = """\
   ******
  *......*
  *.....xxx
xx*....xoxox
   *****xxx
  x x x x
"""
image_nyancat_rgb2 = """\
   ******
  *.....xx
x *....xoxox
 x*.....xxx
   ******
   x x x x
"""

image_loituma_head_palette = {
    '*': (0x18,0x06,0x00),
    '.': (0x10,0x03,0x03),
    '-': (1,1,1),
    '=': (0x20,0x01,0x01),
    'v': (0x20,0x00,0x20),
}
image_loituma_head_rgb_open = """\
 *****
**....*
*.-.-.*
*.....*
**===**
*vvvvv 
"""
image_loituma_head_rgb_closed = """\
 *****
**....*
*.-.-.*
*.....*
**...**
*vvvvv 
"""

image_loituma_leek_palette = {
    'v': (0x00,0x30,0x00),
    '|': (0x20,0x20,0x18),
    'o': (0x20,0x20,0x20),
}

image_loituma_leek_rgb_n = """\
  vvvv
   vv
   vv
   ||
   ||
   ||
   ||
   oo
"""

image_loituma_leek_rgb_ne = """\
    vvvvv
     vvv
     |v
    ||
   ||
   ||
  ||
  oo
"""

image_loituma_leek_rgb_e = """\


       v
o||||vvv
o||||vvv
       v


"""

image_loituma_leek_rgb_se = """\
 oo
 ||
  ||
  ||
   ||
    |v
    vvv
   vvvvv
"""

image_loituma_leek_rgb_s = """\
   oo
   ||
   ||
   ||
   ||
   vv
   vv
  vvvv
"""

image_loituma_leek_rgb_sw = """\
     oo
     ||
    ||
    ||
   ||
  v|
 vvv
vvvvv
"""

image_loituma_leek_rgb_w = """\


       v
o||||vvv
o||||vvv
       v


"""

image_loituma_leek_rgb_nw = """\
vvvvv
 vvv
  v|
   ||
    ||
    ||
     ||
     oo
"""


image_noise_palette = {
    '#': (0x20,0x20,0x20),
}
image_noise_rgb1 = """\
  ####
 #    #
#      #
#  ##  #
#  ##  #
#      #
 #    #
  ####
"""
image_noise_rgb2 = """\
   ####
  #    #
 #      #
#   ##   #
#  #  #  #
#  #  #  #
#   ##   #
 #      #
  #    #
   ####
"""


def main():
    fonts = [
        Font("base", font_base_chars, font_base_bitmap),
        Font("score", font_score_chars, font_score_bitmap),
    ]
    images = [
        Image("bug1", image_bug_palette, image_bug1_rgb),
        Image("bug2", image_bug_palette, image_bug2_rgb),
        Image("nyancat1", image_nyancat_palette, image_nyancat_rgb1),
        Image("nyancat2", image_nyancat_palette, image_nyancat_rgb2),
        Image("loituma_head_open", image_loituma_head_palette, image_loituma_head_rgb_open),
        Image("loituma_head_closed", image_loituma_head_palette, image_loituma_head_rgb_closed),
        Image("loituma_leek_n", image_loituma_leek_palette, image_loituma_leek_rgb_n),
        Image("loituma_leek_ne", image_loituma_leek_palette, image_loituma_leek_rgb_ne),
        Image("loituma_leek_e", image_loituma_leek_palette, image_loituma_leek_rgb_e),
        Image("loituma_leek_se", image_loituma_leek_palette, image_loituma_leek_rgb_se),
        Image("loituma_leek_s", image_loituma_leek_palette, image_loituma_leek_rgb_s),
        Image("loituma_leek_sw", image_loituma_leek_palette, image_loituma_leek_rgb_sw),
        Image("loituma_leek_w", image_loituma_leek_palette, image_loituma_leek_rgb_w),
        Image("loituma_leek_nw", image_loituma_leek_palette, image_loituma_leek_rgb_nw),
        Image("noise1", image_noise_palette, image_noise_rgb1),
        Image("noise2", image_noise_palette, image_noise_rgb2),
    ]

    for font in fonts:
        print("")
        print(font.gen_c())

    for image in images:
        print("")
        print(image.gen_c())


if __name__ == "__main__":
    main()

