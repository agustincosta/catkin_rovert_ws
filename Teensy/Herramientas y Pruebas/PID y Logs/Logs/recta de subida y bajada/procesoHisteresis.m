close all
clear all

data1subybaj = readtable('./Logs/recta de subida y bajada/rueda1ExpIncDec','HeaderLines',1);
data2subybaj = readtable('./Logs/recta de subida y bajada/rueda2ExpIncDec2','HeaderLines',0);
data3subybaj = readtable('./Logs/recta de subida y bajada/rueda3ExpIncDec','HeaderLines',1);
data4subybaj = readtable('./Logs/recta de subida y bajada/rueda4ExpIncDec','HeaderLines',1);

plot(data1subybaj.wi,data1subybaj.wo)

valorActual = 0
auxiliarDePromedio = 0
contador = 0
wi_aux = 0

salidaPromediada = [0 0];

for row=1:size(data1subybaj,1)
    if(data1subybaj.input(row) == valorActual)
        auxiliarDePromedio = auxiliarDePromedio + data1subybaj.wo(row);
        contador = contador + 1;
    else
        if contador == 0
            auxiliarDePromedio = data1subybaj.wo(row)
            contador =1
        end
        
        
        valorActual = data1subybaj.input(row)
        auxiliare = auxiliarDePromedio/contador
        salidaPromediada = [salidaPromediada ; [wi_aux auxiliare]];
        wi_aux= data1subybaj.wi(row)
        auxiliarDePromedio = 0;
        contador = 0;
        
    end
end

plot(salidaPromediada(:,1),salidaPromediada(:,2))