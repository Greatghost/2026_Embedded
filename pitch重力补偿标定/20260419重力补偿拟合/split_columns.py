import csv

# Read the original CSV file
input_file = 'Ozone_DataGraph_260419.csv'

with open(input_file, 'r', encoding='utf-8') as f:
    lines = f.readlines()

# Parse the header
header = lines[0].strip().split(';')
print(f"Columns: {header}")

# Create separate files for each column
column_files = []
for col_name in header:
    # Clean the column name for filename
    safe_name = col_name.replace('.', '_')
    output_file = f'{safe_name}.csv'
    column_files.append(open(output_file, 'w', encoding='utf-8', newline=''))
    column_files[-1].write(f'{col_name}\n')

# Write data to each column file
for line in lines[1:]:
    values = line.strip().split(';')
    for i, value in enumerate(values):
        column_files[i].write(f'{value}\n')

# Close all files
for f in column_files:
    f.close()

print(f"Created {len(header)} separate files:")
for col_name in header:
    safe_name = col_name.replace('.', '_')
    print(f"  - {safe_name}.csv")