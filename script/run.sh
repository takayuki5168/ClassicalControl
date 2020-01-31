SCRIPT_DIR=$(dirname $0)

cd ${SCRIPT_DIR}/../build/
make -j3
./main

if [ $1 == "po" ]; then
    gnuplot -e "
       set xlabel \"time[s]\";
       set ylabel \"output\";
       plot \"output_log.log\" u 1:2 w lp t \"Output\";
       replot \"control_input_log.log\" u 1:2 w lp t \"ControlInput\";pause -1"
elif [ $1 == "popo" ]; then
    gnuplot -e "
       set xlabel \"time[s]\";
       set ylabel \"output\";
       plot \"output_log.log\" u 1:2 w lp t \"Output\";pause -1"
else
    gnuplot -e "
       set xlabel \"time[s]\";
       set ylabel \"output\";
       plot \"output_log.log\" u 1:2 w lp t \"Output\";
       replot \"control_input_log.log\" u 1:2 w lp t \"ControlInput\";
       replot \"disturb_input_log.log\" u 1:2 w lp t \"DisturbInput\"; pause -1"
fi
