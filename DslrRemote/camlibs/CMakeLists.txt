cmake_minimum_required(VERSION 3.12...3.24)

project(libgphoto2_camlibs)

option(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
option(BUILD_SHARED_LIBS "Build shared library." ON)
option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)

#The commented cameras are currently not supported, yet -> CMakeLists.txt missing in corresponding subfolder

# # bla
# option(CAM_adc65 "Build with this CAM." OFF)
# if(CAM_adc65)
        # add_subdirectory(adc65)
# endif(CAM_adc65)

# # bla
# option(CAM_agfa-cl20 "Build with this CAM." OFF)
# if(CAM_agfa-cl20)
        # add_subdirectory(agfa-cl20)
# endif(CAM_agfa-cl20)

# # bla
# option(CAM_aox "Build with this CAM." OFF)
# if(CAM_aox)
        # add_subdirectory(aox)
# endif(CAM_aox)

# # bla
# option(CAM_ax203 "Build with this CAM." OFF)
# if(CAM_ax203)
        # add_subdirectory(ax203)
# endif(CAM_ax203)

# # bla
# option(CAM_barbie "Build with this CAM." OFF)
# if(CAM_barbie)
        # add_subdirectory(barbie)
# endif(CAM_barbie)

# # bla
# option(CAM_canon "Build with this CAM." OFF)
# if(CAM_canon)
        # add_subdirectory(canon)
# endif(CAM_canon)

# # bla
# option(CAM_casio "Build with this CAM." OFF)
# if(CAM_casio)
        # add_subdirectory(casio)
# endif(CAM_casio)

# # bla
# option(CAM_clicksmart310 "Build with this CAM." OFF)
# if(CAM_clicksmart310)
        # add_subdirectory(clicksmart310)
# endif(CAM_clicksmart310)

# # bla
# option(CAM_digigr8 "Build with this CAM." OFF)
# if(CAM_digigr8)
        # add_subdirectory(digigr8)
# endif(CAM_digigr8)

# # bla
# option(CAM_digita "Build with this CAM." OFF)
# if(CAM_digita)
        # add_subdirectory(digita)
# endif(CAM_digita)

# # bla
# option(CAM_dimera "Build with this CAM." OFF)
# if(CAM_dimera)
        # add_subdirectory(dimera)
# endif(CAM_dimera)

# # bla
# option(CAM_directory "Build with this CAM." OFF)
# if(CAM_directory)
        # add_subdirectory(directory)
# endif(CAM_directory)

# # bla
# option(CAM_enigma13 "Build with this CAM." OFF)
# if(CAM_enigma13)
        # add_subdirectory(enigma13)
# endif(CAM_enigma13)

# # bla
# option(CAM_fuji "Build with this CAM." OFF)
# if(CAM_fuji)
        # add_subdirectory(fuji)
# endif(CAM_fuji)

# # bla
# option(CAM_gsmart300 "Build with this CAM." OFF)
# if(CAM_gsmart300)
        # add_subdirectory(gsmart300)
# endif(CAM_gsmart300)

# # bla
# option(CAM_hp215 "Build with this CAM." OFF)
# if(CAM_hp215)
        # add_subdirectory(hp215)
# endif(CAM_hp215)

# # bla
# option(CAM_iclick "Build with this CAM." OFF)
# if(CAM_iclick)
        # add_subdirectory(iclick)
# endif(CAM_iclick)

# # bla
# option(CAM_jamcam "Build with this CAM." OFF)
# if(CAM_jamcam)
        # add_subdirectory(jamcam)
# endif(CAM_jamcam)

# # bla
# option(CAM_jd11 "Build with this CAM." OFF)
# if(CAM_jd11)
        # add_subdirectory(jd11)
# endif(CAM_jd11)

# # bla
# option(CAM_jl2005a "Build with this CAM." OFF)
# if(CAM_jl2005a)
        # add_subdirectory(jl2005a)
# endif(CAM_jl2005a)

# # bla
# option(CAM_jl2005c "Build with this CAM." OFF)
# if(CAM_jl2005c)
        # add_subdirectory(jl2005c)
# endif(CAM_jl2005c)

# # bla
# option(CAM_kodak "Build with this CAM." OFF)
# if(CAM_kodak)
        # add_subdirectory(kodak)
# endif(CAM_kodak)

# # bla
# option(CAM_konica "Build with this CAM." OFF)
# if(CAM_konica)
        # add_subdirectory(konica)
# endif(CAM_konica)

# # bla
# option(CAM_largan "Build with this CAM." OFF)
# if(CAM_largan)
        # add_subdirectory(largan)
# endif(CAM_largan)

# # bla
# option(CAM_lg_gsm "Build with this CAM." OFF)
# if(CAM_lg_gsm)
        # add_subdirectory(lg_gsm)
# endif(CAM_lg_gsm)

# # bla
# option(CAM_mars "Build with this CAM." OFF)
# if(CAM_mars)
        # add_subdirectory(mars)
# endif(CAM_mars)

# # bla
# option(CAM_minolta "Build with this CAM." OFF)
# if(CAM_minolta)
        # add_subdirectory(minolta)
# endif(CAM_minolta)

# # bla
# option(CAM_mustek "Build with this CAM." OFF)
# if(CAM_mustek)
        # add_subdirectory(mustek)
# endif(CAM_mustek)

# # bla
# option(CAM_panasonic "Build with this CAM." OFF)
# if(CAM_panasonic)
        # add_subdirectory(panasonic)
# endif(CAM_panasonic)

# # bla
# option(CAM_pccam300 "Build with this CAM." OFF)
# if(CAM_pccam300)
        # add_subdirectory(pccam300)
# endif(CAM_pccam300)

# # bla
# option(CAM_pccam600 "Build with this CAM." OFF)
# if(CAM_pccam600)
        # add_subdirectory(pccam600)
# endif(CAM_pccam600)

# # bla
# option(CAM_pentax "Build with this CAM." OFF)
# if(CAM_pentax)
        # add_subdirectory(pentax)
# endif(CAM_pentax)

# # bla
# option(CAM_polaroid "Build with this CAM." OFF)
# if(CAM_polaroid)
        # add_subdirectory(polaroid)
# endif(CAM_polaroid)

# bla
option(CAM_ptp2 "Build with this CAM." ON)
if(CAM_ptp2)
        add_subdirectory(ptp2)
endif(CAM_ptp2)

# # bla
# option(CAM_ricoh "Build with this CAM." OFF)
# if(CAM_ricoh)
        # add_subdirectory(ricoh)
# endif(CAM_ricoh)

# # bla
# option(CAM_samsung "Build with this CAM." OFF)
# if(CAM_samsung)
        # add_subdirectory(samsung)
# endif(CAM_samsung)

# # bla
# option(CAM_sierra "Build with this CAM." OFF)
# if(CAM_sierra)
        # add_subdirectory(sierra)
# endif(CAM_sierra)

# # bla
# option(CAM_sipix "Build with this CAM." OFF)
# if(CAM_sipix)
        # add_subdirectory(sipix)
# endif(CAM_sipix)

# # bla
# option(CAM_smal "Build with this CAM." OFF)
# if(CAM_smal)
        # add_subdirectory(smal)
# endif(CAM_smal)

# # bla
# option(CAM_sonix "Build with this CAM." OFF)
# if(CAM_sonix)
        # add_subdirectory(sonix)
# endif(CAM_sonix)

# # bla
# option(CAM_sonydscf1 "Build with this CAM." OFF)
# if(CAM_sonydscf1)
        # add_subdirectory(sonydscf1)
# endif(CAM_sonydscf1)

# # bla
# option(CAM_sonydscf55 "Build with this CAM." OFF)
# if(CAM_sonydscf55)
        # add_subdirectory(sonydscf55)
# endif(CAM_sonydscf55)

# # bla
# option(CAM_soundvision "Build with this CAM." OFF)
# if(CAM_soundvision)
        # add_subdirectory(soundvision)
# endif(CAM_soundvision)

# # bla
# option(CAM_spca50x "Build with this CAM." OFF)
# if(CAM_spca50x)
        # add_subdirectory(spca50x)
# endif(CAM_spca50x)

# # bla
# option(CAM_sq905 "Build with this CAM." OFF)
# if(CAM_sq905)
        # add_subdirectory(sq905)
# endif(CAM_sq905)

# # bla
# option(CAM_st2205 "Build with this CAM." OFF)
# if(CAM_st2205)
        # add_subdirectory(st2205)
# endif(CAM_st2205)

# # bla
# option(CAM_stv0674 "Build with this CAM." OFF)
# if(CAM_stv0674)
        # add_subdirectory(stv0674)
# endif(CAM_stv0674)

# # bla
# option(CAM_stv0680 "Build with this CAM." OFF)
# if(CAM_stv0680)
        # add_subdirectory(stv0680)
# endif(CAM_stv0680)

# # bla
# option(CAM_sx330z "Build with this CAM." OFF)
# if(CAM_sx330z)
        # add_subdirectory(sx330z)
# endif(CAM_sx330z)

# # bla
# option(CAM_topfield "Build with this CAM." OFF)
# if(CAM_topfield)
        # add_subdirectory(topfield)
# endif(CAM_topfield)

# # bla
# option(CAM_toshiba "Build with this CAM." OFF)
# if(CAM_toshiba)
        # add_subdirectory(toshiba)
# endif(CAM_toshiba)

# # bla
# option(CAM_tp6801 "Build with this CAM." OFF)
# if(CAM_tp6801)
        # add_subdirectory(tp6801)
# endif(CAM_tp6801)
