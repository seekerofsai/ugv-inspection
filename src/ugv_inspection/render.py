"""Small dependency-free RGB/PNG renderer for labelled synthetic demo artifacts."""
import struct
import zlib

WIDTH, HEIGHT = 480, 320
COLORS = [(70, 215, 175), (255, 185, 75), (125, 160, 255)]


def rgb_frame(boxes, target, fresh=True):
    bg = (20, 30, 44) if fresh else (55, 25, 34)
    data = bytearray(bytes(bg) * WIDTH * HEIGHT)

    def rect(x0, y0, x1, y1, color):
        x0, x1 = max(0, int(x0)), min(WIDTH, int(x1))
        y0, y1 = max(0, int(y0)), min(HEIGHT, int(y1))
        if x1 <= x0 or y1 <= y0:
            return
        row = bytes(color) * (x1-x0)
        for y in range(y0, y1):
            data[(y*WIDTH+x0)*3:(y*WIDTH+x1)*3] = row

    def label(text, x, y):
        glyphs = {
            'S':['11111','10000','10000','11111','00001','00001','11111'],
            'I':['111','010','010','010','010','010','111'],
            'M':['10001','11011','10101','10101','10001','10001','10001'],
            'U':['10001','10001','10001','10001','10001','10001','11111'],
            'L':['10000','10000','10000','10000','10000','10000','11111'],
            'A':['01110','10001','10001','11111','10001','10001','10001'],
            'T':['11111','00100','00100','00100','00100','00100','00100'],
            'O':['01110','10001','10001','10001','10001','10001','01110'],
            'N':['10001','11001','11001','10101','10011','10011','10001']}
        for ch in text:
            rows = glyphs[ch]
            for iy,row in enumerate(rows):
                for ix,pixel in enumerate(row):
                    if pixel == '1': rect(x+ix*2,y+iy*2,x+ix*2+2,y+iy*2+2,(190,205,220))
            x += (len(rows[0])+1)*2

    for x in range(0, WIDTH, 40):
        rect(x, 0, x+1, HEIGHT, (35, 47, 62))
    for y in range(0, HEIGHT, 40):
        rect(0, y, WIDTH, y+1, (35, 47, 62))
    if fresh:
        for key, box in boxes.items():
            key = int(key)
            x, y, w, h = box
            cx, cy = (x+1)*WIDTH/2, (y+1)*HEIGHT/2
            hw, hh = w*WIDTH/2, h*HEIGHT/2
            color = COLORS[key % len(COLORS)]
            rect(cx-hw, cy-hh, cx+hw, cy+hh, color)
            rect(cx-hw+3, cy-hh+3, cx+hw-3, cy+hh-3, (40, 55, 70))
            # Distinct inspection features: switch / gauge / connector.
            if key % 3 == 0:
                rect(cx-5, cy-10, cx+5, cy+10, color)
            elif key % 3 == 1:
                rect(cx-10, cy-2, cx+10, cy+2, color)
                rect(cx-2, cy-10, cx+2, cy+2, color)
            else:
                for k in (-8, 0, 8):
                    rect(cx+k-2, cy-6, cx+k+2, cy+6, color)
    rect(WIDTH//2-12, HEIGHT//2, WIDTH//2+13, HEIGHT//2+1, (240,240,240))
    rect(WIDTH//2, HEIGHT//2-12, WIDTH//2+1, HEIGHT//2+13, (240,240,240))
    # Status strip: selected target color, camera availability.
    rect(0, HEIGHT-8, WIDTH, HEIGHT, COLORS[target % len(COLORS)] if fresh else (230,70,90))
    label("SIMULATION", 12, 12)
    return bytes(data)


def png_bytes(rgb, width=WIDTH, height=HEIGHT):
    def chunk(kind, data):
        return struct.pack('!I', len(data)) + kind + data + struct.pack('!I', zlib.crc32(kind+data)&0xffffffff)
    raw = b''.join(b'\0'+rgb[y*width*3:(y+1)*width*3] for y in range(height))
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('!2I5B', width,height,8,2,0,0,0))
            + chunk(b'IDAT', zlib.compress(raw)) + chunk(b'IEND', b''))
