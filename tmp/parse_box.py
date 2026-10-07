import xml.etree.ElementTree as ET
p='250902/250902/Untitled2/POUs/MAIN.TcPOU'; root=ET.parse(p).getroot()
for box in root.iter():
 if box.attrib.get('t')=='BoxTreeBox' and any((v.text or '').strip('"')=='MC_Power' for v in box.findall('v')):
  print('BOX')
  for e in box.iter():
   if e.attrib.get('n') in ('InputItems','Names','Operand','Type'):
    print(e.tag,e.attrib,repr(e.text))
  break
