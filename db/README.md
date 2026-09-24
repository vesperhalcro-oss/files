# PostgreSQL setup

Tactical Mapper uses PostgreSQL for asynchronous persistence.

Create a database named `tactical_mapper_db`, then run `schema.sql`:

```powershell
createdb -h 127.0.0.1 -p 5432 -U postgres tactical_mapper_db
psql -h 127.0.0.1 -p 5432 -U postgres -d tactical_mapper_db -f db/schema.sql
```

The application reads `TACTICAL_MAPPER_DB_URL`. For a local default, it uses:

```text
host=127.0.0.1 port=5432 user=postgres dbname=tactical_mapper_db
```

Set the environment variable when a password or a different local role is
required. Database failures are surfaced in the GUI as `Storage: degraded`.
