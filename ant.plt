set terminal qt size 600,600

set style line 12 lc rgb'#808080' lt 0 lw 1
set grid back ls 12

set xtics nomirror rotate by -45
set format x "%9.0f"
set yrange [20:180]
plot 'ant.dat' using 1:2
