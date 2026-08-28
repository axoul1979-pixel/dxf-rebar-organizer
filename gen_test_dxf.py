import pathlib
FIX = pathlib.Path(__file__).resolve().parent / "fixtures"

import sys

def pair(c, v): return "%3d\n%s\n" % (c, v)

def block(name, ents):
    s = pair(0,"BLOCK") + pair(8,"0") + pair(2,name) + pair(70,0)
    s += pair(10,"0.0") + pair(20,"0.0") + pair(30,"0.0") + pair(3,name) + pair(1,"")
    s += ents
    s += pair(0,"ENDBLK") + pair(8,"0")
    return s

def line(x1,y1,x2,y2,layer="STRUCT"):
    return pair(0,"LINE")+pair(8,layer)+pair(10,repr(x1))+pair(20,repr(y1))+pair(30,"0.0")+ \
           pair(11,repr(x2))+pair(21,repr(y2))+pair(31,"0.0")

MODE = "mtext"

def mtext(x,y,txt,h=0.18,rot=0.0,layer="TEXT"):
    if MODE == "text":
        return pair(0,"TEXT")+pair(8,layer)+pair(10,repr(x))+pair(20,repr(y))+pair(30,"0.0")+ \
               pair(40,repr(h))+pair(1,txt)+pair(50,repr(rot))+pair(7,"STANDARD")
    return pair(0,"MTEXT")+pair(8,layer)+pair(10,repr(x))+pair(20,repr(y))+pair(30,"0.0")+ \
           pair(40,repr(h))+pair(41,"5.0")+pair(71,5)+pair(72,5)+pair(7,"STANDARD")+ \
           pair(1,txt)+pair(50,repr(rot))

def rect(x0,y0,x1,y1):
    return line(x0,y0,x1,y0)+line(x1,y0,x1,y1)+line(x1,y1,x0,y1)+line(x0,y1,x0,y0)

def make(lvl, x0, y0, w, h, extra_shared_block_body=None, path=None):
    L = str(lvl)
    s = pair(0,"SECTION")+pair(2,"HEADER")
    s += pair(9,"$ACADVER")+pair(1,"AC1009")
    s += pair(9,"$INSBASE")+pair(10,"0.0")+pair(20,"0.0")+pair(30,"0.0")
    s += pair(9,"$EXTMIN")+pair(10,repr(x0))+pair(20,repr(y0))+pair(30,"0.0")
    s += pair(9,"$EXTMAX")+pair(10,repr(x0+w))+pair(20,repr(y0+h))+pair(30,"0.0")
    s += pair(9,"$LIMMIN")+pair(10,"0.0")+pair(20,"0.0")
    s += pair(9,"$LIMMAX")+pair(10,"100.0")+pair(20,"100.0")
    s += pair(0,"ENDSEC")

    s += pair(0,"SECTION")+pair(2,"TABLES")
    s += pair(0,"TABLE")+pair(2,"LTYPE")+pair(70,1)
    s += pair(0,"LTYPE")+pair(2,"CONTINUOUS")+pair(70,0)+pair(3,"Solid line")+pair(72,65)+pair(73,0)+pair(40,"0.0")
    s += pair(0,"ENDTAB")
    s += pair(0,"TABLE")+pair(2,"LAYER")+pair(70,3)
    for nm, col in (("0",7),("STRUCT",7),("TEXT",2),("L%s_ONLY"%L,3)):
        s += pair(0,"LAYER")+pair(2,nm)+pair(70,0)+pair(62,col)+pair(6,"CONTINUOUS")
    s += pair(0,"ENDTAB")
    s += pair(0,"TABLE")+pair(2,"STYLE")+pair(70,1)
    s += pair(0,"STYLE")+pair(2,"STANDARD")+pair(70,0)+pair(40,"0.0")+pair(41,"1.0")+pair(50,"0.0")+ \
         pair(71,0)+pair(42,"0.2")+pair(3,"txt")+pair(4,"")
    s += pair(0,"ENDTAB")
    s += pair(0,"ENDSEC")

    s += pair(0,"SECTION")+pair(2,"BLOCKS")
    s += block("FL%s_BEAM1"%L, rect(x0,y0,x0+w,y0+h))
    s += block("FL%s_COLUMN1"%L, rect(x0+1,y0+1,x0+1.6,y0+1.6))
    s += block("FL%s_BEAMBAR1"%L, mtext(x0+2, y0+2, "\\U+03A610/20 \\U+0391\\U+03BD\\U+03C9"))
    s += block("FL%s_BEAM1_TEXT"%L, mtext(x0+3, y0+3, "\\U+03941"))
    # κοινό block με ΙΔΙΟ όνομα σε όλες τις στάθμες
    s += block("ARROWHEAD", extra_shared_block_body or (line(0,0,0.2,0.1)+line(0,0,0.2,-0.1)))
    s += pair(0,"ENDSEC")

    s += pair(0,"SECTION")+pair(2,"ENTITIES")
    for nm in ("FL%s_BEAM1"%L,"FL%s_COLUMN1"%L,"FL%s_BEAMBAR1"%L,"FL%s_BEAM1_TEXT"%L):
        s += pair(0,"INSERT")+pair(8,"0")+pair(2,nm)+pair(10,"0.0")+pair(20,"0.0")+pair(30,"0.0")
    s += pair(0,"INSERT")+pair(8,"0")+pair(2,"ARROWHEAD")+pair(10,repr(x0+4))+pair(20,repr(y0+4))+pair(30,"0.0")
    # γυμνή οντότητα εκτός block
    s += line(x0, y0+h+1, x0+w, y0+h+1, "TEXT")
    s += pair(0,"CIRCLE")+pair(8,"TEXT")+pair(10,repr(x0+5))+pair(20,repr(y0+5))+pair(30,"0.0")+pair(40,"0.25")
    s += pair(0,"ENDSEC")
    s += pair(0,"EOF")
    open(path,"w",encoding="latin-1").write(s)

def build(outdir):
    import os
    os.makedirs(outdir, exist_ok=True)
    make(-2,   0.0,   0.0, 12.0, 8.0, path=outdir+"/KTIRIO_A_or-2_tidied.dxf")
    make(-1,  50.0,  30.0, 14.0, 9.0, path=outdir+"/KTIRIO_A_or-1_tidied.dxf")
    make(0,  100.0, 200.0, 12.5, 8.5, path=outdir+"/KTIRIO_A_or0_tidied.dxf")
    make(1, -300.0, -80.0, 13.0, 7.0, path=outdir+"/KTIRIO_A_or1_tidied.dxf")
    # στάθμη 2 με ΔΙΑΦΟΡΕΤΙΚΟ κοινό block ίδιου ονόματος -> δοκιμή σύγκρουσης
    make(2,   10.0,  10.0, 11.0, 6.0,
         extra_shared_block_body=line(0,0,0.9,0.4)+line(0,0,0.9,-0.4),
         path=outdir+"/KTIRIO_A_or2_tidied.dxf")

if __name__ == "__main__":
    build(str(FIX/"r12_mtext"))
    MODE = "text"
    build(str(FIX/"r12_text"))
    print("ok")
