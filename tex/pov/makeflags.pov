
#LDIR = +L/usr/lib/povray-3.5/ +L/usr/lib/povray-3.5/include +L../povray-include
LDIR = +L/usr/local/share/povray-3.6 +L/usr/local/share/povray-3.6/include +L../povray-include
POVRAY = povray +FT -D $(LDIR)


ARG0=+H120 +W160
ARG1=+H240 +W320
ARG2=+H480 +W640
ARG3=+W640 +H480 slow.ini
ARG4=+H768 +W1024 slow.ini
ARG5=+H960 +W1280 slow.ini
ARG6=+H1500 +W2000 slow.ini
ARG7=+H3000 +W4000 slow.ini

%.jpg: %.4.tga
	convert -trim -quality 95 $< $@

%.tga: %.pov
	$(POVRAY)  +I$*.pov +O$@ $(ARG0);
%.1.tga: %.pov
	$(POVRAY)  +I$*.pov +O$@ $(ARG1);
%.2.tga: %.pov
	$(POVRAY)  +I$*.pov +O$@ $(ARG2);
%.3.tga: %.pov
	$(POVRAY)  +I$*.pov +O$@ $(ARG3);
%.4.tga: %.pov
	$(POVRAY)  +I$*.pov +O$@ $(ARG4);
%.5.tga: %.pov
	$(POVRAY)  +I$*.pov +O$@ $(ARG5);
%.6.tga: %.pov
	$(POVRAY)  +I$*.pov +O$@ $(ARG6);
%.7.tga: %.pov
	$(POVRAY)  +I$*.pov +O$@ $(ARG7);

%.tga: %.pov.gz
	gunzip -c $*.pov.gz | $(POVRAY)  +I- +O$@ $(ARG0);
%.1.tga: %.pov.gz
	gunzip -c $*.pov.gz | $(POVRAY)  +I- +O$@ $(ARG1);
%.2.tga: %.pov.gz
	gunzip -c $*.pov.gz | $(POVRAY)  +I- +O$@ $(ARG2);
%.3.tga: %.pov.gz
	gunzip -c $*.pov.gz | $(POVRAY)  +I- +O$@ $(ARG3);
%.4.tga: %.pov.gz
	gunzip -c $*.pov.gz | $(POVRAY)  +I- +O$@ $(ARG4);
%.5.tga: %.pov.gz
	gunzip -c $*.pov.gz | $(POVRAY)  +I- +O$@ $(ARG5);
%.6.tga: %.pov.gz
	gunzip -c $*.pov.gz | $(POVRAY)  +I- +O$@ $(ARG6);
%.7.tga: %.pov.gz
	gunzip -c $*.pov.gz | $(POVRAY)  +I- +O$@ $(ARG7);

#SRCFILES=`ls *.pov | sed 's/\.pov//g'`
#TGAFILES=`ls *.pov | sed 's/\.pov/.\*.tga/g'`
