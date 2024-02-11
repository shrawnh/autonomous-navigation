clean:
	rm -rf venv

create-env-win: # to activate the environment: source venv/Scripts/activate 
	python -m venv venv
	venv\Scripts\pip install -r requirements.txt

create-env-mac: # to activate the environment: source venv/bin/activate
	python -m venv venv
	venv/bin/pip install -r requirements.txt

freeze:
	pip freeze > requirements.txt