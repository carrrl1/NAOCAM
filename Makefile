#Reglas a ejecutar cuando se ejecute make
all: Doxygen
Doxygen:
	doxygen Doxyfile

#Al digitar make clean se borraran todos los archivos compilados,
#note e l −f en rm, s i no sabe para que sirve use e l manual de rm.
