import sys
import xml.etree.ElementTree as ET
from pathlib import Path
p=Path(sys.argv[1])
print(p)
root=ET.parse(p).getroot()
print(root.tag, len(list(root.iter())))
for e in list(root.iter())[:50]: print(e.tag,e.attrib)
