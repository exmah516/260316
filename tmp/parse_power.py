import sys
import xml.etree.ElementTree as ET
from pathlib import Path
paths = [Path(sys.argv[1]), Path(sys.argv[2])]
for path in paths:
 print("---",path)
 root=ET.parse(path).getroot()
 for box in root.iter():
  if box.attrib.get('t') != 'BoxTreeBox': continue
  typ=inst=None
  for v in box:
   if v.tag=='v' and v.attrib.get('n')=='BoxType': typ=(v.text or '').strip('"')
   if v.tag=='o' and v.attrib.get('n')=='Instance':
    for vv in v:
     if vv.tag=='v' and vv.attrib.get('n')=='Operand': inst=(vv.text or '').strip('"')
  if typ=='MC_Power':
   print('MC_Power', inst)
   for o in box.iter('o'):
    if o.attrib.get('n')=='InputItems':
     for ii in o.iter('o'):
      vals=[]
      for vv in ii.iter('v'):
       if vv.attrib.get('n') in ('Operand','Type'): vals.append(vv.attrib.get('n')+'='+str(vv.text))
      if vals: print('  '+' '.join(vals))
