#!/usr/bin/env python3

import string
from PIL import Image,ImageDraw,ImageFont


def main():
  
  fnt = ImageFont.truetype('/usr/share/fonts/truetype/freefont/FreeMono.ttf',7)

  w,h = 6,6
  dpxls = {}
  for c in string.printable:
    im = Image.new('RGB',(w,h))
    d = ImageDraw.Draw(im)
    d.text((0,0), c, font=fnt)

    ms = []
    for j in range(0,h):
      for i in range(0,w):
        r,g,b = im.getpixel((i,j))
        m = (r+g+b)/3
        ms.append(m)
    dpxls[c] = ms

  lut = []
  pixels = []
  for ic in range(0,127):
    c = chr(ic)
    if c in dpxls:
      v = dpxls[c]
      pixels.append(v)
      lut.append(len(pixels)-1)
    else:
      lut.append(-1)
  print("const uint8_t font_width = %s;"%w);
  print("const uint8_t font_height = %s;"%h);
  print("const int8_t font_pixels_lut[] = {%s};"%(','.join(["%d"%i for i in lut])))
  print("const uint8_t font_pixels[][%d*%d] PROGMEM = {"%(w,h))
  for ls in pixels:
    print("{%s},"%(','.join(["%d"%i for i in ls])))
  print("};")

if __name__ == "__main__":
  main()
