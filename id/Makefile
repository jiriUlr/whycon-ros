CFLAGS := -ggdb -std=c++11 -Wall -O2 -march=native
CFLAGS += `Magick++-config --cxxflags --cppflags`
LDFLAGS := CNecklace.cpp -lm
LDFLAGS += `Magick++-config --ldflags --libs`

all: whycon_id_gen

whycon_id_gen: whycon_id_gen.cpp CNecklace.cpp
	$(CXX) whycon_id_gen.cpp $(CFLAGS) $(LDFLAGS) -o $@ 

clean:
	rm -f *.o whycon_id_gen

.PHONY: clean

