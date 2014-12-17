#Reglas a ejecutar cuando se ejecute make
all: Doxygen ProyectoFinal.pdf
ProyectoFinal.pdf:
ProyectoFinal.pdf : ProyectoFinal.tex
	pdflatex ProyectoFinal.tex 
Doxygen:
	doxygen Doxyfile
	
clean:
	rm -f *.log *.aux *.gz

#Al digitar make clean se borraran todos los archivos compilados,
#note e l −f en rm, s i no sabe para que sirve use e l manual de rm.
