import re

def remove_points_with_x_greater_than_8(input_filename, output_filename):
    with open(input_filename, 'r') as file:
        lines = file.readlines()

    header = True
    header_lines = []
    points = []
    for line in lines:
        if header:
            header_lines.append(line)
            if re.match(r"^DATA\s+ascii$", line.strip(), re.IGNORECASE):
                header = False
            continue
        parts = line.strip().split()
        if len(parts) >= 3:
            x, y, z = map(float, parts[:3])
            if x <= 8:
                points.append([x, y, z])

    # 更新点数信息
    for i, line in enumerate(header_lines):
        if line.startswith('POINTS'):
            header_lines[i] = f'POINTS {len(points)}\n'
        elif line.startswith('DATA'):
            header_lines[i] = 'DATA ascii\n'

    # 保存结果到新的PCD文件
    with open(output_filename, 'w') as file:
        for line in header_lines:
            file.write(line)
        for point in points:
            file.write(f"{point[0]} {point[1]} {point[2]}\n")

# 使用示例
remove_points_with_x_greater_than_8('map.pcd', 'map1.pcd')
