clean:
	rm -rf venv

create-env-win:
	python -m venv venv
	venv\Scripts\pip install -r requirements.txt

note-source-win:
	source venv/Scripts/activate

create-env-make:
	python -m venv venv
	venv/bin/pip install -r requirements.txt

source-mac:
	source venv/bin/activate