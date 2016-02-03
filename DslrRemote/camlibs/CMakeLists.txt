project(libgphoto2_camlibs)

cmake_minimum_required(VERSION 3.0)

OPTION(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
OPTION(BUILD_SHARED_LIBS "Build shared library." ON)
OPTION(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." OFF)
SET(ITOM_DIR "" CACHE PATH "base path to itom") 

# bla
OPTION(CAM_adc65 "Build with this CAM." OFF)
if (CAM_adc65)
		ADD_SUBDIRECTORY(adc65)
endif (CAM_adc65)

# bla
OPTION(CAM_agfa-cl20 "Build with this CAM." OFF)
if (CAM_agfa-cl20)
		ADD_SUBDIRECTORY(agfa-cl20)
endif (CAM_agfa-cl20)

# bla
OPTION(CAM_aox "Build with this CAM." OFF)
if (CAM_aox)
		ADD_SUBDIRECTORY(aox)
endif (CAM_aox)

# bla
OPTION(CAM_ax203 "Build with this CAM." OFF)
if (CAM_ax203)
		ADD_SUBDIRECTORY(ax203)
endif (CAM_ax203)

# bla
OPTION(CAM_barbie "Build with this CAM." OFF)
if (CAM_barbie)
		ADD_SUBDIRECTORY(barbie)
endif (CAM_barbie)

# bla
OPTION(CAM_canon "Build with this CAM." OFF)
if (CAM_canon)
		ADD_SUBDIRECTORY(canon)
endif (CAM_canon)

# bla
OPTION(CAM_casio "Build with this CAM." OFF)
if (CAM_casio)
		ADD_SUBDIRECTORY(casio)
endif (CAM_casio)

# bla
OPTION(CAM_clicksmart310 "Build with this CAM." OFF)
if (CAM_clicksmart310)
		ADD_SUBDIRECTORY(clicksmart310)
endif (CAM_clicksmart310)

# bla
OPTION(CAM_digigr8 "Build with this CAM." OFF)
if (CAM_digigr8)
		ADD_SUBDIRECTORY(digigr8)
endif (CAM_digigr8)

# bla
OPTION(CAM_digita "Build with this CAM." OFF)
if (CAM_digita)
		ADD_SUBDIRECTORY(digita)
endif (CAM_digita)

# bla
OPTION(CAM_dimera "Build with this CAM." OFF)
if (CAM_dimera)
		ADD_SUBDIRECTORY(dimera)
endif (CAM_dimera)

# bla
OPTION(CAM_directory "Build with this CAM." OFF)
if (CAM_directory)
		ADD_SUBDIRECTORY(directory)
endif (CAM_directory)

# bla
OPTION(CAM_enigma13 "Build with this CAM." OFF)
if (CAM_enigma13)
		ADD_SUBDIRECTORY(enigma13)
endif (CAM_enigma13)

# bla
OPTION(CAM_fuji "Build with this CAM." OFF)
if (CAM_fuji)
		ADD_SUBDIRECTORY(fuji)
endif (CAM_fuji)

# bla
OPTION(CAM_gsmart300 "Build with this CAM." OFF)
if (CAM_gsmart300)
		ADD_SUBDIRECTORY(gsmart300)
endif (CAM_gsmart300)

# bla
OPTION(CAM_hp215 "Build with this CAM." OFF)
if (CAM_hp215)
		ADD_SUBDIRECTORY(hp215)
endif (CAM_hp215)

# bla
OPTION(CAM_iclick "Build with this CAM." OFF)
if (CAM_iclick)
		ADD_SUBDIRECTORY(iclick)
endif (CAM_iclick)

# bla
OPTION(CAM_jamcam "Build with this CAM." OFF)
if (CAM_jamcam)
		ADD_SUBDIRECTORY(jamcam)
endif (CAM_jamcam)

# bla
OPTION(CAM_jd11 "Build with this CAM." OFF)
if (CAM_jd11)
		ADD_SUBDIRECTORY(jd11)
endif (CAM_jd11)

# bla
OPTION(CAM_jl2005a "Build with this CAM." OFF)
if (CAM_jl2005a)
		ADD_SUBDIRECTORY(jl2005a)
endif (CAM_jl2005a)

# bla
OPTION(CAM_jl2005c "Build with this CAM." OFF)
if (CAM_jl2005c)
		ADD_SUBDIRECTORY(jl2005c)
endif (CAM_jl2005c)

# bla
OPTION(CAM_kodak "Build with this CAM." OFF)
if (CAM_kodak)
		ADD_SUBDIRECTORY(kodak)
endif (CAM_kodak)

# bla
OPTION(CAM_konica "Build with this CAM." OFF)
if (CAM_konica)
		ADD_SUBDIRECTORY(konica)
endif (CAM_konica)

# bla
OPTION(CAM_largan "Build with this CAM." OFF)
if (CAM_largan)
		ADD_SUBDIRECTORY(largan)
endif (CAM_largan)

# bla
OPTION(CAM_lg_gsm "Build with this CAM." OFF)
if (CAM_lg_gsm)
		ADD_SUBDIRECTORY(lg_gsm)
endif (CAM_lg_gsm)

# bla
OPTION(CAM_mars "Build with this CAM." OFF)
if (CAM_mars)
		ADD_SUBDIRECTORY(mars)
endif (CAM_mars)

# bla
OPTION(CAM_minolta "Build with this CAM." OFF)
if (CAM_minolta)
		ADD_SUBDIRECTORY(minolta)
endif (CAM_minolta)

# bla
OPTION(CAM_mustek "Build with this CAM." OFF)
if (CAM_mustek)
		ADD_SUBDIRECTORY(mustek)
endif (CAM_mustek)

# bla
OPTION(CAM_panasonic "Build with this CAM." OFF)
if (CAM_panasonic)
		ADD_SUBDIRECTORY(panasonic)
endif (CAM_panasonic)

# bla
OPTION(CAM_pccam300 "Build with this CAM." OFF)
if (CAM_pccam300)
		ADD_SUBDIRECTORY(pccam300)
endif (CAM_pccam300)

# bla
OPTION(CAM_pccam600 "Build with this CAM." OFF)
if (CAM_pccam600)
		ADD_SUBDIRECTORY(pccam600)
endif (CAM_pccam600)

# bla
OPTION(CAM_pentax "Build with this CAM." OFF)
if (CAM_pentax)
		ADD_SUBDIRECTORY(pentax)
endif (CAM_pentax)

# bla
OPTION(CAM_polaroid "Build with this CAM." OFF)
if (CAM_polaroid)
		ADD_SUBDIRECTORY(polaroid)
endif (CAM_polaroid)

# bla
OPTION(CAM_ptp2 "Build with this CAM." ON)
if (CAM_ptp2)
		ADD_SUBDIRECTORY(ptp2)
endif (CAM_ptp2)

# bla
OPTION(CAM_ricoh "Build with this CAM." OFF)
if (CAM_ricoh)
		ADD_SUBDIRECTORY(ricoh)
endif (CAM_ricoh)

# bla
OPTION(CAM_samsung "Build with this CAM." OFF)
if (CAM_samsung)
		ADD_SUBDIRECTORY(samsung)
endif (CAM_samsung)

# bla
OPTION(CAM_sierra "Build with this CAM." OFF)
if (CAM_sierra)
		ADD_SUBDIRECTORY(sierra)
endif (CAM_sierra)

# bla
OPTION(CAM_sipix "Build with this CAM." OFF)
if (CAM_sipix)
		ADD_SUBDIRECTORY(sipix)
endif (CAM_sipix)

# bla
OPTION(CAM_smal "Build with this CAM." OFF)
if (CAM_smal)
		ADD_SUBDIRECTORY(smal)
endif (CAM_smal)

# bla
OPTION(CAM_sonix "Build with this CAM." OFF)
if (CAM_sonix)
		ADD_SUBDIRECTORY(sonix)
endif (CAM_sonix)

# bla
OPTION(CAM_sonydscf1 "Build with this CAM." OFF)
if (CAM_sonydscf1)
		ADD_SUBDIRECTORY(sonydscf1)
endif (CAM_sonydscf1)

# bla
OPTION(CAM_sonydscf55 "Build with this CAM." OFF)
if (CAM_sonydscf55)
		ADD_SUBDIRECTORY(sonydscf55)
endif (CAM_sonydscf55)

# bla
OPTION(CAM_soundvision "Build with this CAM." OFF)
if (CAM_soundvision)
		ADD_SUBDIRECTORY(soundvision)
endif (CAM_soundvision)

# bla
OPTION(CAM_spca50x "Build with this CAM." OFF)
if (CAM_spca50x)
		ADD_SUBDIRECTORY(spca50x)
endif (CAM_spca50x)

# bla
OPTION(CAM_sq905 "Build with this CAM." OFF)
if (CAM_sq905)
		ADD_SUBDIRECTORY(sq905)
endif (CAM_sq905)

# bla
OPTION(CAM_st2205 "Build with this CAM." OFF)
if (CAM_st2205)
		ADD_SUBDIRECTORY(st2205)
endif (CAM_st2205)

# bla
OPTION(CAM_stv0674 "Build with this CAM." OFF)
if (CAM_stv0674)
		ADD_SUBDIRECTORY(stv0674)
endif (CAM_stv0674)

# bla
OPTION(CAM_stv0680 "Build with this CAM." OFF)
if (CAM_stv0680)
		ADD_SUBDIRECTORY(stv0680)
endif (CAM_stv0680)

# bla
OPTION(CAM_sx330z "Build with this CAM." OFF)
if (CAM_sx330z)
		ADD_SUBDIRECTORY(sx330z)
endif (CAM_sx330z)

# bla
OPTION(CAM_topfield "Build with this CAM." OFF)
if (CAM_topfield)
		ADD_SUBDIRECTORY(topfield)
endif (CAM_topfield)

# bla
OPTION(CAM_toshiba "Build with this CAM." OFF)
if (CAM_toshiba)
		ADD_SUBDIRECTORY(toshiba)
endif (CAM_toshiba)

# bla
OPTION(CAM_tp6801 "Build with this CAM." OFF)
if (CAM_tp6801)
		ADD_SUBDIRECTORY(tp6801)
endif (CAM_tp6801)