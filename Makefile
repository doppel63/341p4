CC=vcs

FLAGS=-sverilog -debug -assert filter -assert enable_diag

default: public

student: top.sv tb.sv usbHost.sv thumb.sv.e 
	$(CC) $(FLAGS) top.sv tb.sv usbHost.sv thumb.sv.e

public: top.sv TA_tb.sv usbHost.sv thumb.sv.e
	$(CC) $(FLAGS) top.sv TA_tb.sv usbHost.sv thumb.sv.e

faulty: top.sv TA_tb_faults.sv usbHost.sv thumb_faulty.sv.e
	$(CC) $(FLAGS) top.sv TA_tb_faults.sv usbHost.sv thumb_faulty.sv.e

prelab: top.sv prelab_tb.sv usbHost.sv prelab_thumb.sv.e
	$(CC) $(FLAGS) top.sv prelab_tb.sv usbHost.sv prelab_thumb.sv.e

test: top.sv test_tb.sv usbHost.sv thumb.sv.e
	$(CC) $(FLAGS) top.sv test_tb.sv usbHost.sv thumb.sv.e

clean:
	rm -rf simv
	rm -rf simv.daidir
	rm -rf csrc
	rm -rf ucli.key
	rm -rf simv.vdb
	rm -rf DVEfiles
	rm -rf inter.vpd
