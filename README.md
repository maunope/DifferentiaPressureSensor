# differentialBarometer
A small Arduino project to record data from an I2C differential barometer


**BAROMETRO DIFFERENZIALE**

Serve a misurare la differenza di pressione fra un ambiente in aria libera e uno confinato.

Nel nostro caso la differenza di pressione atmosferica presente all’esterno della grotta e la pressione all’interna della stessa.

La grotta viene sigillata.

Lo strumento resta all’esterno.

Lo strumento ha due sensori di pressione.

Un sensore deve essere collegato a un tubicino di qualche decina di centimetri che possa raggiungere la zona confinata.

Serve un sistema di allineamento dei due sensori che li riporti allo zero virtuale, interessa solo la differenza di pressione, non i valori di pressione presenti.

Sono consigliati sensori con scala compresa entro 50 Pa.

(Esiste un prototipo con 2 coppie di sensori 50 Pa e 100 Pa ma non ha dato alcun vantaggio… anzi)

La precisione richiesta è di 1 Pa (la pressione di un decimo di millimetro di acqua al livello del mare)

Molto utile, direi indispensabile,  un piccolo schermo che indichi l’avvenuto allineamento a zero,  il differenziale in tempo reale, e l’attivazione del data logger (vedi sotto)

Sono stati costruiti prototipi che contengono lettori di temperatura, umidità e CO2 ma è preferibile dedicare uno strumento a ciascun parametro. Il sistema risulta più versatile. Il troppo guasta!

**DATA LOGGER**

Serve un registratore programmabile per eseguire una lettura puntuale ogni 60 – 120 secondi (se si prevede un sistema piccolo può servire una lettura ogni 30 secondi, per sistemi molto grandi ogni 180 secondi, ma sono casi limite difficilmente utilizzati)

La lettura deve essere puntuale, non servono registrazioni prolungate perché i valori non cambiano mai tanto rapidamente.

Il data logger deve poter registrare data ora e valori per almeno 24 ore.

Sono stati fatti studi particolari su intere aree carsiche molto complesse che hanno richiesto registrazioni annuali, queste ricerche non rientrano attualmente nei nostri obiettivi.

!!!ATT!!! All’esterno della grotta viene posto un lettore della pressione atmosferica per meglio interpretare i valori letti. Qui possiamo aprire una discussione sulla opportunità o meno di introdurre un allineamento automatico dei sensori ogni 300 o 600 secondi .

https://docs.google.com/spreadsheets/d/1zlWzPYfntZh6GAmRLwcTypwWLMDD_adUwLnR_UYPygM/edit?gid=1617350203#gid=1617350203
