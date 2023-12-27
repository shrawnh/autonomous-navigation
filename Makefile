clean:
	rm -rf venv

create-env:
	python -m venv venv
	venv\Scripts\pip install -r requirements.txt

note-source:
	source venv/Scripts/activate