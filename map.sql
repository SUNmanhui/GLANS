-- Table: public.roadpath

-- DROP TABLE public.roadpath;

CREATE TABLE public.roadpath
(
  gid integer NOT NULL DEFAULT nextval('roadpath_gid_seq'::regclass),
  x numeric(10,0),
  y numeric(10,0),
  start numeric(10,0),
  "end" numeric(10,0),
  start_x numeric,
  start_y numeric,
  end_x numeric,
  end_y numeric,
  geom geometry(MultiLineString,4326),
  source integer,
  target integer,
  length double precision,
  reverse_cost double precision,
  CONSTRAINT roadpath_pkey PRIMARY KEY (gid)
)
WITH (
  OIDS=FALSE
);
ALTER TABLE public.roadpath
  OWNER TO postgres;

-- Index: public.roadpath_geom_gidx

-- DROP INDEX public.roadpath_geom_gidx;

CREATE INDEX roadpath_geom_gidx
  ON public.roadpath
  USING gist
  (geom);

-- Index: public.roadpath_source_idx

-- DROP INDEX public.roadpath_source_idx;

CREATE INDEX roadpath_source_idx
  ON public.roadpath
  USING btree
  (source);

-- Index: public.roadpath_target_idx

-- DROP INDEX public.roadpath_target_idx;

CREATE INDEX roadpath_target_idx
  ON public.roadpath
  USING btree
  (target);

-- Index: public.source_idx1

-- DROP INDEX public.source_idx1;

CREATE INDEX source_idx1
  ON public.roadpath
  USING btree
  (source);

-- Index: public.target_idx1

-- DROP INDEX public.target_idx1;

CREATE INDEX target_idx1
  ON public.roadpath
  USING btree
  (target);


