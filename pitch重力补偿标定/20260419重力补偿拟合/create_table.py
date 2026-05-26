import csv

# Read the original CSV file
input_file = 'Ozone_DataGraph_260419.csv'
output_file = 'angle_current_table.csv'

with open(input_file, 'r', encoding='utf-8') as f:
    reader = csv.reader(f, delimiter=';')
    header = next(reader)  # Skip header

    # Create new CSV with angle and current columns
    with open(output_file, 'w', encoding='utf-8', newline='') as out:
        writer = csv.writer(out)
        writer.writerow(['角度 (angle)', '电流 (current)'])

        for row in reader:
            if len(row) >= 3:
                angle = row[1]
                current = row[2]
                writer.writerow([angle, current])

print(f"Created {output_file}")
print("\nPreview (first 10 rows):")
with open(output_file, 'r', encoding='utf-8') as f:
    for i, line in enumerate(f):
        if i < 11:
            print(line.strip())