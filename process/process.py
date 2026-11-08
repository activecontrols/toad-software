with open('ripple.log') as f:
    lines = f.readlines()

rows = []
for line in lines:
    if 'PT Readings:' in line:
        rows.append([0, 0, 0, 0])
    if 'PT_N2_01_tank' in line:
        rows[-1][0] = (float(line.split()[1]))
    if 'PT_N2_02_reg' in line:
        rows[-1][1] = (float(line.split()[1]))
    if 'PT_N2_03_service' in line:
        rows[-1][2] = (float(line.split()[1]))
    if 'PT_O2_01_tank' in line:
        rows[-1][3] = (float(line.split()[1]))

with open('out.csv', 'w+') as f:
    f.write('time, pt1 V, pt2 V, pt3 V, pt4 V\n')
    for idx, row in enumerate(rows):
        f.write(str(idx) + ", " + ", ".join(map(str, row)) + '\n')

