#declare NV= 308+1; #declare NF=264;
#declare V=array[NV] {<0,0,0>,
<0.362873,0.0731539,0.120958>,
<0.16226,0.044606,0.250766>,
<0.177879,0.031641,0.177879>,
<0.232611,0.0541079,-0.232611>,
<0.19955,0.0464174,-0.23025>,
<0.171665,0.0399312,-0.224485>,
<0.271112,0.0630637,-0.229403>,
<0.360654,0.0838921,-0.194198>,
<0.31449,0.0731539,-0.217724>,
<0.405271,0.0942705,-0.155873>,
<0.4417,0.102744,-0.101931>,
<0.462486,0.107579,-0.0355758>,
<0.462486,0.107579,0.0355758>,
<0.4417,0.102744,0.101931>,
<0.360654,0.0838921,0.194198>,
<0.405271,0.0942705,0.155873>,
<0.31449,0.0731539,0.217724>,
<0.271112,0.0630637,0.229403>,
<0.232611,0.0541079,0.232611>,
<0.171665,0.0399312,0.224485>,
<0.192274,0.0528569,0.262192>,
<-0.177879,0.031641,-0.177879>,
<-0.200027,0.0355807,-0.176495>,
<-0.224485,0.0399312,-0.171665>,
<-0.250766,0.044606,-0.16226>,
<-0.277876,0.0494283,-0.147111>,
<-0.304184,0.0541079,-0.125252>,
<-0.327433,0.0582435,-0.0963039>,
<-0.345014,0.0613707,-0.0608848>,
<-0.354531,0.0630637,-0.0208548>,
<-0.354531,0.0630637,0.0208548>,
<-0.345014,0.0613707,0.0608848>,
<-0.327433,0.0582435,0.0963039>,
<-0.304184,0.0541079,0.125252>,
<-0.277876,0.0494283,0.147111>,
<-0.250766,0.044606,0.16226>,
<-0.224485,0.0399312,0.171665>,
<-0.200027,0.0355807,0.176495>,
<-0.177879,0.031641,0.177879>,
<0.19955,0.0464174,0.23025>,
<0.229403,0.0630637,0.271112>,
<-0.176495,0.0355807,-0.200027>,
<-0.201596,0.0406411,-0.201596>,
<-0.23025,0.0464174,-0.19955>,
<-0.262192,0.0528569,-0.192274>,
<-0.296465,0.0597663,-0.177879>,
<-0.331089,0.0667463,-0.154508>,
<-0.362873,0.0731539,-0.120958>,
<-0.387685,0.0781559,-0.077537>,
<-0.401408,0.0809224,-0.0267606>,
<-0.401408,0.0809224,0.0267606>,
<-0.387685,0.0781559,0.077537>,
<-0.362873,0.0731539,0.120958>,
<-0.331089,0.0667463,0.154508>,
<-0.296465,0.0597663,0.177879>,
<-0.262192,0.0528569,0.192274>,
<-0.23025,0.0464174,0.19955>,
<-0.201596,0.0406411,0.201596>,
<-0.176495,0.0355807,0.200027>,
<0.200027,0.0355807,0.176495>,
<0.274904,0.0755722,0.274904>,
<-0.171665,0.0399312,-0.224485>,
<-0.19955,0.0464174,-0.23025>,
<-0.232611,0.0541079,-0.232611>,
<-0.271112,0.0630637,-0.229403>,
<-0.31449,0.0731539,-0.217724>,
<-0.360654,0.0838921,-0.194198>,
<-0.405271,0.0942705,-0.155873>,
<-0.4417,0.102744,-0.101931>,
<-0.462486,0.107579,-0.0355758>,
<-0.462486,0.107579,0.0355758>,
<-0.4417,0.102744,0.101931>,
<-0.405271,0.0942705,0.155873>,
<-0.360654,0.0838921,0.194198>,
<-0.31449,0.0731539,0.217724>,
<-0.271112,0.0630637,0.229403>,
<-0.232611,0.0541079,0.232611>,
<-0.19955,0.0464174,0.23025>,
<-0.171665,0.0399312,0.224485>,
<0.331089,0.0667463,0.154508>,
<0.391334,0.107579,0.249031>,
<-0.16226,0.044606,-0.250766>,
<-0.192274,0.0528569,-0.262192>,
<-0.229403,0.0630637,-0.271112>,
<-0.274904,0.0755722,-0.274904>,
<-0.32934,0.090537,-0.26946>,
<-0.391334,0.107579,-0.249031>,
<-0.455663,0.125264,-0.207119>,
<-0.511744,0.140681,-0.139567>,
<-0.545301,0.149906,-0.0495729>,
<-0.545301,0.149906,0.0495729>,
<-0.511744,0.140681,0.139567>,
<-0.455663,0.125264,0.207119>,
<-0.391334,0.107579,0.249031>,
<-0.32934,0.090537,0.26946>,
<-0.274904,0.0755722,0.274904>,
<-0.229403,0.0630637,0.271112>,
<-0.192274,0.0528569,0.262192>,
<-0.16226,0.044606,0.250766>,
<0.176495,0.0355807,-0.200027>,
<0.455663,0.125264,0.207119>,
<-0.147111,0.0494283,-0.277876>,
<-0.177879,0.0597663,-0.296465>,
<-0.217724,0.0731539,-0.31449>,
<-0.26946,0.090537,-0.32934>,
<-0.335994,0.112892,-0.335994>,
<-0.4187,0.140681,-0.325655>,
<-0.5135,0.172533,-0.285278>,
<-0.604789,0.203205,-0.201596>,
<-0.663793,0.22303,-0.0737547>,
<-0.663793,0.22303,0.0737547>,
<-0.604789,0.203205,0.201596>,
<-0.5135,0.172533,0.285278>,
<-0.4187,0.140681,0.325655>,
<-0.335994,0.112892,0.335994>,
<-0.26946,0.090537,0.32934>,
<-0.217724,0.0731539,0.31449>,
<-0.177879,0.0597663,0.296465>,
<-0.147111,0.0494283,0.277876>,
<0.201596,0.0406411,-0.201596>,
<0.32934,0.090537,0.26946>,
<-0.125252,0.0541079,-0.304184>,
<-0.154508,0.0667463,-0.331089>,
<-0.194198,0.0838921,-0.360654>,
<-0.249031,0.107579,-0.391334>,
<-0.325655,0.140681,-0.4187>,
<-0.431992,0.186617,-0.431992>,
<-0.572097,0.247142,-0.408641>,
<-0.729918,0.315319,-0.312822>,
<-0.846704,0.365769,-0.120958>,
<-0.846704,0.365769,0.120958>,
<-0.729918,0.315319,0.312822>,
<-0.572097,0.247142,0.408641>,
<-0.431992,0.186617,0.431992>,
<-0.325655,0.140681,0.4187>,
<-0.249031,0.107579,0.391334>,
<-0.194198,0.0838921,0.360654>,
<-0.154508,0.0667463,0.331089>,
<-0.125252,0.0541079,0.304184>,
<0.23025,0.0464174,-0.19955>,
<0.327433,0.0582435,-0.0963039>,
<-0.0963039,0.0582435,-0.327433>,
<-0.120958,0.0731539,-0.362873>,
<-0.155873,0.0942705,-0.405271>,
<-0.207119,0.125264,-0.455663>,
<-0.285278,0.172533,-0.5135>,
<-0.408641,0.247142,-0.572097>,
<-0.604789,0.365769,-0.604789>,
<-0.889395,0.537896,-0.533637>,
<-1.16306,0.703403,-0.232611>,
<-1.16306,0.703403,0.232611>,
<-0.889395,0.537896,0.533637>,
<-0.604789,0.365769,0.604789>,
<-0.408641,0.247142,0.572097>,
<-0.285278,0.172533,0.5135>,
<-0.207119,0.125264,0.455663>,
<-0.155873,0.0942705,0.405271>,
<-0.120958,0.0731539,0.362873>,
<-0.0963039,0.0582435,0.327433>,
<0.262192,0.0528569,-0.192274>,
<0.345014,0.0613707,-0.0608848>,
<-0.0608848,0.0613707,-0.345014>,
<-0.077537,0.0781559,-0.387685>,
<-0.101931,0.102744,-0.4417>,
<-0.139567,0.140681,-0.511744>,
<-0.201596,0.203205,-0.604789>,
<-0.312822,0.315319,-0.729918>,
<-0.533637,0.537896,-0.889395>,
<0.354531,0.0630637,-0.0208548>,
<0.201596,0.0406411,0.201596>,
<0.23025,0.0464174,0.19955>,
<0.176495,0.0355807,0.200027>,
<-0.533637,0.537896,0.889395>,
<-0.312822,0.315319,0.729918>,
<-0.201596,0.203205,0.604789>,
<-0.139567,0.140681,0.511744>,
<-0.101931,0.102744,0.4417>,
<-0.077537,0.0781559,0.387685>,
<-0.0608848,0.0613707,0.345014>,
<0.296465,0.0597663,-0.177879>,
<0.354531,0.0630637,0.0208548>,
<-0.0208548,0.0630637,-0.354531>,
<-0.0267606,0.0809224,-0.401408>,
<-0.0355758,0.107579,-0.462486>,
<-0.0495729,0.149906,-0.545301>,
<-0.0737547,0.22303,-0.663793>,
<-0.120958,0.365769,-0.846704>,
<-0.232611,0.703403,-1.16306>,
<0.296465,0.0597663,0.177879>,
<0.277876,0.0494283,0.147111>,
<-0.232611,0.703403,1.16306>,
<-0.120958,0.365769,0.846704>,
<-0.0737547,0.22303,0.663793>,
<-0.0495729,0.149906,0.545301>,
<-0.0355758,0.107579,0.462486>,
<-0.0267606,0.0809224,0.401408>,
<-0.0208548,0.0630637,0.354531>,
<0.331089,0.0667463,-0.154508>,
<0.327433,0.0582435,0.0963039>,
<0.0208548,0.0630637,-0.354531>,
<0.0267606,0.0809224,-0.401408>,
<0.0355758,0.107579,-0.462486>,
<0.0495729,0.149906,-0.545301>,
<0.0737547,0.22303,-0.663793>,
<0.120958,0.365769,-0.846704>,
<0.232611,0.703403,-1.16306>,
<0.177879,0.031641,-0.177879>,
<0.200027,0.0355807,-0.176495>,
<0.232611,0.703403,1.16306>,
<0.120958,0.365769,0.846704>,
<0.0737547,0.22303,0.663793>,
<0.0495729,0.149906,0.545301>,
<0.0355758,0.107579,0.462486>,
<0.0267606,0.0809224,0.401408>,
<0.0208548,0.0630637,0.354531>,
<0.362873,0.0731539,-0.120958>,
<0.345014,0.0613707,0.0608848>,
<0.0608848,0.0613707,-0.345014>,
<0.077537,0.0781559,-0.387685>,
<0.101931,0.102744,-0.4417>,
<0.139567,0.140681,-0.511744>,
<0.201596,0.203205,-0.604789>,
<0.312822,0.315319,-0.729918>,
<0.533637,0.537896,-0.889395>,
<0.224485,0.0399312,-0.171665>,
<0.250766,0.044606,-0.16226>,
<0.277876,0.0494283,-0.147111>,
<0.304184,0.0541079,-0.125252>,
<0.533637,0.537896,0.889395>,
<0.312822,0.315319,0.729918>,
<0.201596,0.203205,0.604789>,
<0.139567,0.140681,0.511744>,
<0.101931,0.102744,0.4417>,
<0.077537,0.0781559,0.387685>,
<0.0608848,0.0613707,0.345014>,
<0.387685,0.0781559,-0.077537>,
<0.304184,0.0541079,0.125252>,
<0.0963039,0.0582435,-0.327433>,
<0.120958,0.0731539,-0.362873>,
<0.155873,0.0942705,-0.405271>,
<0.207119,0.125264,-0.455663>,
<0.285278,0.172533,-0.5135>,
<0.408641,0.247142,-0.572097>,
<0.604789,0.365769,-0.604789>,
<0.889395,0.537896,-0.533637>,
<1.16306,0.703403,-0.232611>,
<1.16306,0.703403,0.232611>,
<0.889395,0.537896,0.533637>,
<0.604789,0.365769,0.604789>,
<0.408641,0.247142,0.572097>,
<0.285278,0.172533,0.5135>,
<0.207119,0.125264,0.455663>,
<0.155873,0.0942705,0.405271>,
<0.120958,0.0731539,0.362873>,
<0.0963039,0.0582435,0.327433>,
<0.401408,0.0809224,-0.0267606>,
<0.224485,0.0399312,0.171665>,
<0.125252,0.0541079,-0.304184>,
<0.154508,0.0667463,-0.331089>,
<0.194198,0.0838921,-0.360654>,
<0.249031,0.107579,-0.391334>,
<0.325655,0.140681,-0.4187>,
<0.431992,0.186617,-0.431992>,
<0.572097,0.247142,-0.408641>,
<0.729918,0.315319,-0.312822>,
<0.846704,0.365769,-0.120958>,
<0.846704,0.365769,0.120958>,
<0.729918,0.315319,0.312822>,
<0.572097,0.247142,0.408641>,
<0.431992,0.186617,0.431992>,
<0.325655,0.140681,0.4187>,
<0.249031,0.107579,0.391334>,
<0.194198,0.0838921,0.360654>,
<0.154508,0.0667463,0.331089>,
<0.125252,0.0541079,0.304184>,
<0.401408,0.0809224,0.0267606>,
<0.250766,0.044606,0.16226>,
<0.147111,0.0494283,-0.277876>,
<0.177879,0.0597663,-0.296465>,
<0.217724,0.0731539,-0.31449>,
<0.26946,0.090537,-0.32934>,
<0.335994,0.112892,-0.335994>,
<0.4187,0.140681,-0.325655>,
<0.5135,0.172533,-0.285278>,
<0.604789,0.203205,-0.201596>,
<0.663793,0.22303,-0.0737547>,
<0.663793,0.22303,0.0737547>,
<0.604789,0.203205,0.201596>,
<0.5135,0.172533,0.285278>,
<0.4187,0.140681,0.325655>,
<0.335994,0.112892,0.335994>,
<0.26946,0.090537,0.32934>,
<0.217724,0.0731539,0.31449>,
<0.177879,0.0597663,0.296465>,
<0.147111,0.0494283,0.277876>,
<0.387685,0.0781559,0.077537>,
<0.262192,0.0528569,0.192274>,
<0.16226,0.044606,-0.250766>,
<0.192274,0.0528569,-0.262192>,
<0.229403,0.0630637,-0.271112>,
<0.274904,0.0755722,-0.274904>,
<0.32934,0.090537,-0.26946>,
<0.391334,0.107579,-0.249031>,
<0.455663,0.125264,-0.207119>,
<0.511744,0.140681,-0.139567>,
<0.545301,0.149906,-0.0495729>,
<0.545301,0.149906,0.0495729>,
<0.511744,0.140681,0.139567>,}