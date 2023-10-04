# Set the output file format and filename
set terminal pngcairo enhanced color size 800,600
set output 'output.png'

# Set plot title and axis labels
set title 'XY Plot from CSV Data'
set xlabel 'X-Axis Label'
set ylabel 'Y-Axis Label'

# Plot the data from the CSV file
plot 'logger_2023-10-04_12:23:22.csv' using 1:2 with linespoints title 'Data Series'