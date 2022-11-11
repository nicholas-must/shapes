.PHONY: clean install

libshapes.a: shapes.cpp shapes.hpp
	g++ -c shapes.cpp -o libshapes.a

clean:
	@rm libshapes.a 2>/dev/null
	@rm *~ 2>/dev/null

install:
	cp ./libshapes.a /usr/local/lib/
